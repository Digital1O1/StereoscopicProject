#include <iomanip>
#include <iostream>
#include <string.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>
#include <memory>
#include <thread>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include "image.h"
#include "event_loop.h"
#include <X11/Xlib.h>
#include <string>

int main()
{
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cameraMatrixLeft, distCoeffsLeft,
                      cameraMatrixRight, distCoeffsRight,
                      leftCameraRGB.size(), rotationMatrix, translationVector,
                      R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1, leftCameraRGB.size(), 0, 0);

    cv::Mat map1Left, map2Left, map1Right, map2Right;

    cv::initUndistortRectifyMap(cameraMatrixLeft, distCoeffsLeft, R1, P1, leftCameraRGB.size(), CV_32FC1, map1Left, map2Left);
    cv::initUndistortRectifyMap(cameraMatrixRight, distCoeffsRight, R2, P2, rightCameraRGB.size(), CV_32FC1, map1Right, map2Right);

    cv::Mat rectifiedLeft, rectifiedRight;
    cv::remap(leftCameraRGB, rectifiedLeft, map1Left, map2Left, cv::INTER_LINEAR);
    cv::remap(rightCameraRGB, rectifiedRight, map1Right, map2Right, cv::INTER_LINEAR);

    cv::Mat overlayedImage;
    double alpha = 0.5; // Blending factor
    cv::addWeighted(rectifiedLeft, alpha, rectifiedRight, 1 - alpha, 0.0, overlayedImage);

    cv::imshow("Overlayed Image", overlayedImage);
    cv::waitKey(0);
    cv::imwrite("overlayed_image.png", overlayedImage);

    return 0;
}
