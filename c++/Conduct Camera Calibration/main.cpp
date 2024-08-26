/*
    References
        - https://stackoverflow.com/questions/22037558/opencv-camera-calibration-generate-very-distorted-images
*/

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

#define NOIR_CAMERA 0
#define VISIBLE_CAMERA 1
#define CHESSBOARD_ROWS 6
#define CHESSBOARD_COLUMNS 9
#define CHESSBOARD_SIZE cv::Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS)
#define NUMBER_OF_IMAGES 20
#define CAPTURE_DELAY 100
void calibrateCamera(int cameraId, const char *cameraName);

int main()
{
    calibrateCamera(NOIR_CAMERA, "NOIR Camera");
    calibrateCamera(VISIBLE_CAMERA, "Visible Camera");

    return 0;
}

void calibrateCamera(int cameraId, const char *cameraName)
{
    const int CHESSBOARD[2]{CHESSBOARD_ROWS, CHESSBOARD_COLUMNS};
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point3f> objp;

    for (int i = 0; i < CHESSBOARD[1]; i++)
    {
        for (int j = 0; j < CHESSBOARD[0]; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corners;

    cv::VideoCapture cap(cameraId);
    if (!cap.isOpened())
    {
        std::cerr << "Error opening " << cameraName << std::endl;
        return;
    }

    bool previewFlag = true;
    std::cout << "Press 'q or Q' to continue with calibration for " << cameraName << "\n";

    while (previewFlag)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Empty frame" << std::endl;
            continue;
        }
        cv::imshow("Preview window for " + std::string(cameraName), frame);

        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 'Q')
        {
            previewFlag = false;
        }
    }
    previewFlag = true;
    std::string closePreview = "Preview window for " + std::string(cameraName);
    cv::destroyWindow(closePreview);

    std::cout << "\nCapturing " << NUMBER_OF_IMAGES << " chessboard images now. Rotate chessboard slowly for " << cameraName << "\n\n";

    int imagesCaptured = 0;
    while (imagesCaptured < NUMBER_OF_IMAGES)
    {
        cap >> frame;
        if (frame.empty())
        {
            break;
        }
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, CHESSBOARD_SIZE, corners, cv::CALIB_CB_ADAPTIVE_THRESH || cv::CALIB_CB_NORMALIZE_IMAGE || cv::CALIB_CB_FAST_CHECK);
        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            cv::drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, found);
            imagePoints.push_back(corners);
            objectPoints.push_back(objp);
            imagesCaptured++;
            std::cout << "Number of images captured for " << cameraName << ": [" << imagesCaptured << "]\n";
        }
        else
        {
            printf("Image not captured \r\n");
        }

        cv::imshow("Captured Image For Calibration - " + std::string(cameraName), frame);
        if (cv::waitKey(CAPTURE_DELAY) == 27)
        {
            std::cout << "\nExiting program now...\n";
            break;
        }
    }
    printf("\nGenerating camera calibration values and distortion values\r\n");

    std::string windowName = "Captured Image For Calibration - " + std::string(cameraName);
    cv::destroyWindow(windowName);

    // cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::Mat R, T;
    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat distCoeffs(1, 5, CV_64F);

    cv::calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, R, T);

    std::cout << std::endl
              << cameraName << " Matrix:\n\n"
              << cameraMatrix << std::endl;
    std::cout << std::endl
              << cameraName << " Distortion Coefficients:\n\n"
              << distCoeffs << std::endl;

    // Preview image before testing out camera calibration
    std::cout
        << "\nImage preview, press Q or q to test camera calibration"
        << std::endl;

    while (previewFlag)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Empty frame" << std::endl;
            continue;
        }
        cv::imshow("Preview Window for Undistorted image", frame);

        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 'Q')
        {
            previewFlag = false;
        }
    }
    cv::destroyWindow("Preview Window for Undistorted image");

    // Capture image
    cv::Mat capturedImage;
    cap >> capturedImage;

    // Undistort the captured image
    cv::Mat undistortedImage;
    cv::undistort(capturedImage, undistortedImage, cameraMatrix, distCoeffs);

    // Display the original and undistorted images
    cv::imshow("Captured Image " + std::string(cameraName), capturedImage);
    cv::imshow("Undistorted Image " + std::string(cameraName), undistortedImage);

    // Wait for a key press before exiting
    cv::waitKey(0);

    cap.release();
    cv::destroyAllWindows();
}
