#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

int main()
{
    bool previewFlag = true;
    cv::Mat frame, frame2, gray;

    // To determine camera name use : libcamera-hello --list
    // To determine supported color spaces for IMX219 use : gst-inspect-1.0 videoconvert

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

    cv::VideoCapture cap(visibleCameraPipeline, cv::CAP_GSTREAMER);
    cv::VideoCapture cap2(irCameraPipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened() || (!cap2.isOpened()))
    {
        std::cerr << "Error opening " << visibleCameraPipeline << std::endl;
    }
    while (previewFlag)
    {
        cap >> frame;
        cap2 >> frame2;
        if (frame.empty() || frame2.empty())
        {
            std::cerr << "Empty frame" << std::endl;
            continue;
        }
        cv::imshow("Camera0", frame);
        cv::imshow("Camera1", frame2);

        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 'Q')
        {
            previewFlag = false;
        }
    }
    return 0;
}