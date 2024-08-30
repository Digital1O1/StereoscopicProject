#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

// Function to find and display chessboard corners
bool findChessboardCornersAndDisplay(const cv::Mat &image, cv::Size boardSize, std::vector<cv::Point2f> &corners)
{
    bool found = cv::findChessboardCorners(image, boardSize, corners);
    if (found)
    {
        cv::drawChessboardCorners(image, boardSize, corners, found);
        cv::imshow("Chessboard", image);
        cv::waitKey(500); // Display each image for 500ms
    }
    return found;
}

int main()
{
    // Define the size of the chessboard (number of inner corners per a chessboard row and column)
    cv::Size boardSize = cv::Size(9, 6); // Adjust to your chessboard pattern

    // Prepare object points (e.g., (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0))
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            objectPoints.push_back(cv::Point3f(j, i, 0));

    // Vectors to hold the object points and image points from all the images
    std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
    std::vector<std::vector<cv::Point3f>> objectPointsList;

    // Load stereo image pairs and find chessboard corners
    for (int i = 0; i < 10; ++i)
    { // Assume you have 10 image pairs
        std::string leftImagePath = "left" + std::to_string(i) + ".jpg";
        std::string rightImagePath = "right" + std::to_string(i) + ".jpg";

        cv::Mat leftImage = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
        cv::Mat rightImage = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);

        if (leftImage.empty() || rightImage.empty())
        {
            std::cerr << "Failed to load images: " << leftImagePath << " or " << rightImagePath << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners1, corners2;
        bool found1 = findChessboardCornersAndDisplay(leftImage, boardSize, corners1);
        bool found2 = findChessboardCornersAndDisplay(rightImage, boardSize, corners2);

        if (found1 && found2)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            objectPointsList.push_back(objectPoints);
        }
    }

    // Load the camera matrices and distortion coefficients (these should have been pre-calculated)
    cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2;
    cv::FileStorage fs("calibration_data.yml", cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["cameraMatrix1"] >> cameraMatrix1;
        fs["cameraMatrix2"] >> cameraMatrix2;
        fs["distCoeffs1"] >> distCoeffs1;
        fs["distCoeffs2"] >> distCoeffs2;
    }
    else
    {
        std::cerr << "Failed to open calibration data file." << std::endl;
        return -1;
    }

    // Stereo Calibration
    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        objectPointsList, imagePoints1, imagePoints2,
        cameraMatrix1, distCoeffs1,
        cameraMatrix2, distCoeffs2,
        leftImage.size(), R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));

    std::cout << "Stereo Calibration done with RMS error = " << rms << std::endl;

    // Save the calibration results
    cv::FileStorage fsOut("stereo_calibration.yml", cv::FileStorage::WRITE);
    fsOut << "R" << R;
    fsOut << "T" << T;
    fsOut << "E" << E;
    fsOut << "F" << F;

    std::cout << "Stereo calibration data saved to stereo_calibration.yml" << std::endl;

    return 0;
}
