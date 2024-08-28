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

int main()
{
    const int CHESSBOARD[2]{CHESSBOARD_ROWS, CHESSBOARD_COLUMNS}; // Number of inner corners per a chessboard row and column
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

    cv::VideoCapture cap(NOIR_CAMERA); // Open the default camera
    if (!cap.isOpened())
    { // Check if camera opened successfully
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // Preview camera before running calibration

    bool previewFlag = true;
    printf("Press 'q' to continue with calibration\r\n");

    while (previewFlag)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Empty frame" << std::endl;
        }
        cv::imshow("Preview window", frame);

        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 'Q')
        {
            previewFlag = false;
            // break;
        }
    }
    printf("\nCAPTURING 20 CHESSBOARD IMAGES NOW ROTATE CHESSBOARD SLOWLY\r\n");

    int imagesCaptured = 0;
    while (imagesCaptured < 20)
    { // Capture 20 images for calibration
        cap >> frame;
        if (frame.empty())
        {
            break;
        }
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, cv::Size(CHESSBOARD[0], CHESSBOARD[1]), corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            cv::drawChessboardCorners(frame, cv::Size(CHESSBOARD[0], CHESSBOARD[1]), corners, found);
            imagePoints.push_back(corners);
            objectPoints.push_back(objp);
            imagesCaptured++;
            printf("Number of images captured : [ %d ] \r\n", imagesCaptured);
        }
        cv::imshow("Captured Image For Calibration", frame);
        if (cv::waitKey(30) == 27)
        {
            printf("\nExiting program now...\r\n");
            // Exit on pressing 'ESC'
            break;
        }
    }
    printf("\nIMAGES CAPTURED\r\n");

    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, R, T);

    std::cout << "VISIBLE Camera Matrix:\n"
              << cameraMatrix << std::endl;
    std::cout << "VISIBLE Camera Distortion Coefficients:\n"
              << distCoeffs << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
