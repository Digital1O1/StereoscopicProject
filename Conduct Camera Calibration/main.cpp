#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

// Constants
#define CHESSBOARD_ROWS 6
#define CHESSBOARD_COLUMNS 9
#define CHESSBOARD_SIZE cv::Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS)
#define NUMBER_OF_IMAGES 20
#define CAPTURE_DELAY 100

void calibrateCamera(const std::string &cameraPipeline, const std::string &cameraName);

int main()
{
    // GStreamer pipelines
    std::string visibleCameraPipeline = R"(
        libcamerasrc camera-name="/base/soc/i2c0mux/i2c@1/imx219@10" ! 
        video/x-raw,width=640,height=480,framerate=30/1 ! 
        videoconvert ! 
        video/x-raw,format=(string)BGR ! 
        queue ! 
        appsink
    )";

    std::string irCameraPipeline = R"(
        libcamerasrc camera-name="/base/soc/i2c0mux/i2c@0/imx219@10" ! 
        video/x-raw,width=640,height=480,framerate=30/1 ! 
        videoconvert ! 
        video/x-raw,format=(string)BGR ! 
        queue ! 
        appsink
    )";

    // Calibrate both cameras
    calibrateCamera(visibleCameraPipeline, "Visible Camera");
    calibrateCamera(irCameraPipeline, "IR Camera");

    return 0;
}

void calibrateCamera(const std::string &cameraPipeline, const std::string &cameraName)
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point3f> objp;

    // Prepare object points
    for (int i = 0; i < CHESSBOARD_COLUMNS; ++i)
    {
        for (int j = 0; j < CHESSBOARD_ROWS; ++j)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::VideoCapture cap(cameraPipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "Error opening " << cameraName << " pipeline" << std::endl;
        return;
    }

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corners;

    // Preview mode
    std::cout << "Press 'q' to exit the preview for " << cameraName << "\n";
    while (true)
    {
        cap >> frame;
        if (frame.empty())
            continue;

        cv::imshow("Preview - " + cameraName, frame);
        if (cv::waitKey(1) == 'q')
            break;
    }
    cv::destroyWindow("Preview - " + cameraName);

    // Capture chessboard images
    std::cout << "Capturing " << NUMBER_OF_IMAGES << " chessboard images for " << cameraName << "\n";
    int imagesCaptured = 0;

    while (imagesCaptured < NUMBER_OF_IMAGES)
    {
        cap >> frame;
        if (frame.empty())
            break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, CHESSBOARD_SIZE, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH |
                                                   cv::CALIB_CB_NORMALIZE_IMAGE |
                                                   cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            cv::drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, found);

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);
            ++imagesCaptured;
            std::cout << "Captured " << imagesCaptured << " / " << NUMBER_OF_IMAGES << " images.\n";
        }

        cv::imshow("Capturing - " + cameraName, frame);
        if (cv::waitKey(CAPTURE_DELAY) == 27)
            break; // Exit on 'Esc'
    }
    cv::destroyWindow("Capturing - " + cameraName);

    // Calibrate camera
    std::cout << "Calibrating " << cameraName << "...\n";
    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, R, T);

    std::cout << cameraName << " Camera Matrix:\n"
              << cameraMatrix << "\n";
    std::cout << cameraName << " Distortion Coefficients:\n"
              << distCoeffs << "\n";

    // Display undistorted image
    cv::Mat capturedImage, undistortedImage;
    cap >> capturedImage;

    if (!capturedImage.empty())
    {
        cv::undistort(capturedImage, undistortedImage, cameraMatrix, distCoeffs);
        cv::imshow("Original - " + cameraName, capturedImage);
        cv::imshow("Undistorted - " + cameraName, undistortedImage);
        cv::waitKey(0);
    }

    cap.release();
    cv::destroyAllWindows();
}
