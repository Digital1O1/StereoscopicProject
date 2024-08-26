/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Ideas on Board Oy.
 *
 * A simple libcamera capture example
 */

#include <iomanip>
#include <iostream>
#include <string.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>
#include <memory>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "image.h"
#include "event_loop.h"
#include <X11/Xlib.h>

#define TIMEOUT_SEC 50
#define RESOLUTION_WIDTH 640
#define RESOLUTION_LENGTH 480
#define CHESSBOARD_ROWS 6
#define CHESSBOARD_COLUMNS 9
#define CHESSBOARD_SIZE cv::Size(CHESSBOARD_ROWS, CHESSBOARD_COLUMNS)
#define NUMBER_OF_IMAGES 20
#define CAPTURE_DELAY 100

using namespace libcamera;
static std::shared_ptr<Camera> camera;
static EventLoop loop;

int userInput = 0;
int userInputCounter = 1;
int imagesCaptured = 0;
const int CHESSBOARD[2]{CHESSBOARD_ROWS, CHESSBOARD_COLUMNS};
std::vector<std::vector<cv::Point3f>> objectPoints;
std::vector<std::vector<cv::Point2f>> imagePoints;
std::vector<cv::Point3f> objp;
std::vector<cv::Point2f> corners;
int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

std::map<libcamera::FrameBuffer *, std::unique_ptr<Image>> mappedBuffers_;

/*
 * --------------------------------------------------------------------
 * Handle RequestComplete
 *
 * For each Camera::requestCompleted Signal emitted from the Camera the
 * connected Slot is invoked.
 *
 * The Slot is invoked in the CameraManager's thread, hence one should avoid
 * any heavy processing here. The processing of the request shall be re-directed
 * to the application's thread instead, so as not to block the CameraManager's
 * thread for large amount of time.
 *
 * The Slot receives the Request as a parameter.
 */

static void processRequest(Request *request);

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    loop.callLater(std::bind(&processRequest, request));
}

void processPixelData(FrameBuffer *buffer, const StreamConfiguration &cfg, cv::Mat &rgb_image)
{
    Image *img = mappedBuffers_[buffer].get();

    uint8_t *ptr = (uint8_t *)img->data(0).data();

    cv::Mat yData = cv::Mat(cfg.size.height, cfg.size.width, CV_8U, ptr, cfg.stride);
    cv::Mat uData = cv::Mat(cfg.size.height / 2, cfg.size.width / 2, CV_8U, ptr + cfg.size.width * cfg.size.height);
    cv::Mat vData = cv::Mat(cfg.size.height / 2, cfg.size.width / 2, CV_8U, ptr + cfg.size.width * cfg.size.height + cfg.size.width / 2 * cfg.size.height / 2);

    // Resize U and V channels to match Y channel size
    cv::resize(uData, uData, cv::Size(cfg.size.width, cfg.size.height), 0, 0, cv::INTER_LINEAR);
    cv::resize(vData, vData, cv::Size(cfg.size.width, cfg.size.height), 0, 0, cv::INTER_LINEAR);

    std::vector<cv::Mat> yuv_channels = {yData, uData, vData};
    cv::Mat yuv_image;
    cv::merge(yuv_channels, yuv_image);

    cv::cvtColor(yuv_image, rgb_image, cv::COLOR_YUV2BGR);
}

static void processRequest(Request *request)
{
    // std::cout << std::endl
    //           << "Request completed: " << request->toString() << std::endl;

    /*
     * When a request has completed, it is populated with a metadata control
     * list that allows an application to determine various properties of
     * the completed request. This can include the timestamp of the Sensor
     * capture, or its gain and exposure values, or properties from the IPA
     * such as the state of the 3A algorithms.
     *
     * ControlValue types have a toString, so to examine each request, print
     * all the metadata for inspection. A custom application can parse each
     * of these items and process them according to its needs.
     */

    // const ControlList &requestMetadata = request->metadata();
    // for (const auto &ctrl : requestMetadata)
    // {

    //     const ControlId *id = controls::controls.at(ctrl.first);
    //     const ControlValue &value = ctrl.second;

    //     std::cout << "\t" << id->name() << " = " << value.toString()
    //               << std::endl;
    // }

    /*
     * Each buffer has its own FrameMetadata to describe its state, or the
     * usage of each buffer. While in our simple capture we only provide one
     * buffer per request, a request can have a buffer for each stream that
     * is established when configuring the camera.
     *
     * This allows a viewfinder and a still image to be processed at the
     * same time, or to allow obtaining the RAW capture buffer from the
     * sensor along with the image as processed by the ISP.
     */
    const Request::BufferMap &buffers = request->buffers();
    // ----------------------------- [ for (auto bufferPair : buffers) START ] -----------------------------
    for (auto bufferPair : buffers)
    {
        const Stream *stream = bufferPair.first;
        FrameBuffer *buffer = bufferPair.second;

        // const FrameMetadata &metadata = buffer->metadata();

        /* Print some information about the buffer which has completed. */
        // std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence
        //           << " timestamp: " << metadata.timestamp
        //           << " bytesused: ";

        // unsigned int nplane = 0;
        // for (const FrameMetadata::Plane &plane : metadata.planes())
        // {
        //     std::cout << plane.bytesused;
        //     if (++nplane < metadata.planes().size())
        //         std::cout << "/";
        // }

        // std::cout << std::endl;

        StreamConfiguration const &cfg = stream->configuration();

        if (!cfg.colorSpace.has_value())
        {
            std::cout << "Color space is not set." << std::endl;
            exit(1);
        }

        /*
         * Image data can be accessed here, but the FrameBuffer
         * must be mapped by the application
         *
         */

        // Image *img = mappedBuffers_[buffer].get();
        // const libcamera::ColorSpace &colorSpace = libcamera::ColorSpace(cfg.colorSpace.value());
        // std::string colorSpaceStr = colorSpace.toString();
        //  std::cout << "\n --------------------------- [DEBUG INFORMATION ]---------------------------" << std::endl;
        //  std::cout << "\n"
        //            << "Resolution" << " : " << cfg.size.width << " x " << cfg.size.height << std::endl;
        //  std::cout << "Number of bytes in each line of image buffer " << " : " << cfg.stride << std::endl;
        //  std::cout << "pixelFormat" << " : " << cfg.pixelFormat << std::endl;
        //  std::cout << "img->numPlanes() : " << img->numPlanes() << std::endl;
        //  std::cout << "Color Space : " << colorSpaceStr << std::endl;                                                                          // sYCC
        //  std::cout << "colorSpace.Primaries : " << (int)libcamera::ColorSpace::Primaries(colorSpace.primaries) << std::endl;                   // Rec709
        //  std::cout << "colorSpace.TransferFunction : " << (int)libcamera::ColorSpace::YcbcrEncoding(colorSpace.transferFunction) << std::endl; // Srgb
        //  std::cout << "colorSpace.ycbcrEncoding : " << (int)libcamera::ColorSpace::YcbcrEncoding(colorSpace.ycbcrEncoding) << std::endl;       // Rec601
        //  std::cout << "colorSpace.Range : " << (int)libcamera::ColorSpace::YcbcrEncoding(colorSpace.range) << std::endl;                       // Full

        // std::cout << "\n ---------------------------------------------------------------------------" << std::endl;

        cv::Mat previewWindow, gray;
        processPixelData(buffer, cfg, previewWindow);
        cv::namedWindow("PREVIEW WINDOW", cv::WINDOW_AUTOSIZE);
        cv::imshow("PREVIEW WINDOW", previewWindow);
        cv::moveWindow("PREVIEW WINDOW", (1920 - 640) / 2, (1080 - 480) / 2);
        cv::cvtColor(previewWindow, gray, cv::COLOR_BGR2GRAY);

        bool found = findChessboardCorners(gray, CHESSBOARD_SIZE, corners, chessBoardFlags);

        if (found)
        {
            // cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            // cv::drawChessboardCorners(gray, CHESSBOARD_SIZE, corners, found);
            // imagePoints.push_back(corners);
            // objectPoints.push_back(objp);
            imagesCaptured++;
            std::cout << "Number of images captured " << ": [" << imagesCaptured << "]\n";
        }
        else
        {
            printf("Image not captured \r\n");
        }
    }
    // ----------------------------- [ for (auto bufferPair : buffers) END ] -----------------------------

    /* Re-queue the Request to the camera. */
    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}

/*
 * ----------------------------------------------------------------------------
 * Camera Naming.
 *
 * Applications are responsible for deciding how to name cameras, and present
 * that information to the users. Every camera has a unique identifier, though
 * this string is not designed to be friendly for a human reader.
 *
 * To support human consumable names, libcamera provides camera properties
 * that allow an application to determine a naming scheme based on its needs.
 *
 * In this example, we focus on the location property, but also detail the
 * model string for external cameras, as this is more likely to be visible
 * information to the user of an externally connected device.
 *
 * The unique camera ID is appended for informative purposes.
 */
std::string cameraName(Camera *camera)
{
    const ControlList &props = camera->properties();

    std::string name;

    const auto &location = props.get(properties::Location);
    if (location)
    {
        switch (*location)
        {
        case properties::CameraLocationFront:
            name = "Internal front camera";
            break;
        case properties::CameraLocationBack:
            name = "Internal back camera";
            break;
        case properties::CameraLocationExternal:
            name = "External camera";
            const auto &model = props.get(properties::Model);
            if (model)
                name = " '" + *model + "'";
            break;
        }
    }

    name += " (" + camera->id() + ")";

    return name;
}

int main()
{

    /*
     * --------------------------------------------------------------------
     * Create a Camera Manager.
     *
     * The Camera Manager is responsible for enumerating all the Camera
     * in the system, by associating Pipeline Handlers with media entities
     * registered in the system.
     *
     * The CameraManager provides a list of available Cameras that
     * applications can operate on.
     *
     * When the CameraManager is no longer to be used, it should be deleted.
     * We use a unique_ptr here to manage the lifetime automatically during
     * the scope of this function.
     *
     * There can only be a single CameraManager constructed within any
     * process space.
     */
    std::unique_ptr<CameraManager>
        cm = std::make_unique<CameraManager>();
    cm->start();

    /*
     * Just as a test, generate names of the Cameras registered in the
     * system, and list them.
     */
    for (auto const &camera : cm->cameras())
        std::cout << " - " << cameraName(camera.get()) << std::endl;

    /*
     * --------------------------------------------------------------------
     * Camera
     *
     * Camera are entities created by pipeline handlers, inspecting the
     * entities registered in the system and reported to applications
     * by the CameraManager.
     *
     * In general terms, a Camera corresponds to a single image source
     * available in the system, such as an image sensor.
     *
     * Application lock usage of Camera by 'acquiring' them.
     * Once done with it, application shall similarly 'release' the Camera.
     *
     * As an example, use the first available camera in the system after
     * making sure that at least one camera is available.
     *
     * Cameras can be obtained by their ID or their index, to demonstrate
     * this, the following code gets the ID of the first camera; then gets
     * the camera associated with that ID (which is of course the same as
     * cm->cameras()[0]).
     */
    if (cm->cameras().empty())
    {
        std::cout << "No cameras were identified on the system."
                  << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    std::string cameraId = cm->cameras()[0]->id();
    camera = cm->get(cameraId);
    camera->acquire();

    /*
     * Stream
     *
     * Each Camera supports a variable number of Stream. A Stream is
     * produced by processing data produced by an image source, usually
     * by an ISP.
     *
     *   +-------------------------------------------------------+
     *   | Camera                                                |
     *   |                +-----------+                          |
     *   | +--------+     |           |------> [  Main output  ] |
     *   | | Image  |     |           |                          |
     *   | |        |---->|    ISP    |------> [   Viewfinder  ] |
     *   | | Source |     |           |                          |
     *   | +--------+     |           |------> [ Still Capture ] |
     *   |                +-----------+                          |
     *   +-------------------------------------------------------+
     *
     * The number and capabilities of the Stream in a Camera are
     * a platform dependent property, and it's the pipeline handler
     * implementation that has the responsibility of correctly
     * report them.
     */

    /*
     * --------------------------------------------------------------------
     * Camera Configuration.
     *
     * Camera configuration is tricky! It boils down to assign resources
     * of the system (such as DMA engines, scalers, format converters) to
     * the different image streams an application has requested.
     *
     * Depending on the system characteristics, some combinations of
     * sizes, formats and stream usages might or might not be possible.
     *
     * A Camera produces a CameraConfigration based on a set of intended
     * roles for each Stream the application requires.
     */
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({StreamRole::Viewfinder});

    /*
     * The CameraConfiguration contains a StreamConfiguration instance
     * for each StreamRole requested by the application, provided
     * the Camera can support all of them.
     *
     * Each StreamConfiguration has default size and format, assigned
     * by the Camera depending on the Role the application has requested.
     */
    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Default viewfinder configuration is: "
              << streamConfig.toString() << std::endl;

    // for (auto pxlFmts : streamConfig.formats().pixelformats())
    // {
    // 	std::cout << pxlFmts << std::endl;
    // }
    /*
        ------------------- [ NOTES ABOUT PIXEL FORMAT START ] -------------------

        YUV420
            - Color encoding system
            - Commonly used in video compression/transmission
            - Structure
                - Y component : Represents luminance (brightness) of the image
                    - Luminance definiton : measurement of the amount of light emitted/reflected/transmitted by a surface
                    - Corresponds to the perceived brightness of that surface
                - U and V components : Represents chrominance/color information
                    - U --> The difference between blue and luminance
                    - V --> The difference between red and luminance
            - Chroma Subsampling
                - 4:2:0 Subsampling
                    - For every 4 luminance (Y) samples there's 1 chrominance sample for both U and V
            - Advantages
                - Compression efficiency
                    - Reducing amount of chrominance/color data allows for significant data c
                      ompression w/o greatly affecting perceived image quality
                    - Human vision more sensitive to brightness changes vs color changes
            - Disadvantages
                - Reduced color resolution
                    - The chroma subsampling can cause loss of color detail, particularly in areas with sharp color transitions
                - Artifacts
                    - Could introduce artifacts in high-contrast edges due to lower resolution of chrominance information

        ------------------- [ NOTES ABOUT PIXEL FORMAT END ] -------------------

    */

    // Additional supported formats can be found /usr/include/libcamera/libcamera/format.h
    // streamConfig.pixelFormat = formats::YUYV; // 'Column striations'
    streamConfig.pixelFormat = formats::YUV420; // Results in grayscale w/o striations

    streamConfig.size.width = RESOLUTION_WIDTH;
    streamConfig.size.height = RESOLUTION_LENGTH;

    /*
     * Each StreamConfiguration parameter which is part of a
     * CameraConfiguration can be independently modified by the
     * application.
     *
     * In order to validate the modified parameter, the CameraConfiguration
     * should be validated -before- the CameraConfiguration gets applied
     * to the Camera.
     *
     * The CameraConfiguration validation process adjusts each
     * StreamConfiguration to a valid value.
     */

    /*
     * The Camera configuration procedure fails with invalid parameters.
     */
#if 0
	streamConfig.size.width = 0; //4096
	streamConfig.size.height = 0; //2560

	int ret = camera->configure(config.get());
	if (ret) {
		std::cout << "CONFIGURATION FAILED!" << std::endl;
		return EXIT_FAILURE;
	}
#endif

    /*
     * Validating a CameraConfiguration -before- applying it will adjust it
     * to a valid configuration which is as close as possible to the one
     * requested.
     */
    config->validate();
    std::cout << "Validated viewfinder configuration is: "
              << streamConfig.toString() << std::endl;

    /*
     * Once we have a validated configuration, we can apply it to the
     * Camera.
     */
    camera->configure(config.get());

    /*
     * --------------------------------------------------------------------
     * Buffer Allocation
     *
     * Now that a camera has been configured, it knows all about its
     * Streams sizes and formats. The captured images need to be stored in
     * framebuffers which can either be provided by the application to the
     * library, or allocated in the Camera and exposed to the application
     * by libcamera.
     *
     * An application may decide to allocate framebuffers from elsewhere,
     * for example in memory allocated by the display driver that will
     * render the captured frames. The application will provide them to
     * libcamera by constructing FrameBuffer instances to capture images
     * directly into.
     *
     * Alternatively libcamera can help the application by exporting
     * buffers allocated in the Camera using a FrameBufferAllocator
     * instance and referencing a configured Camera to determine the
     * appropriate buffer size and types to create.
     */
    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

    for (StreamConfiguration &cfg : *config)
    {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0)
        {
            std::cerr << "Can't allocate buffers" << std::endl;
            return EXIT_FAILURE;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;

        for (const std::unique_ptr<FrameBuffer> &buffer : allocator->buffers(cfg.stream()))
        {
            std::unique_ptr<Image> image = Image::fromFrameBuffer(buffer.get(), Image::MapMode::ReadOnly);
            assert(image != nullptr);
            mappedBuffers_[buffer.get()] = std::move(image);
        }
    }

    /*
     * --------------------------------------------------------------------
     * Frame Capture
     *
     * libcamera frames capture model is based on the 'Request' concept.
     * For each frame a Request has to be queued to the Camera.
     *
     * A Request refers to (at least one) Stream for which a Buffer that
     * will be filled with image data shall be added to the Request.
     *
     * A Request is associated with a list of Controls, which are tunable
     * parameters (similar to v4l2_controls) that have to be applied to
     * the image.
     *
     * Once a request completes, all its buffers will contain image data
     * that applications can access and for each of them a list of metadata
     * properties that reports the capture parameters applied to the image.
     */
    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;
    for (unsigned int i = 0; i < buffers.size(); ++i)
    {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return EXIT_FAILURE;
        }

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << "Can't set buffer for request"
                      << std::endl;
            return EXIT_FAILURE;
        }

        /*
         * Controls can be added to a request on a per frame basis.
         */
        ControlList &controls = request->controls();
        // controls.set(controls::Brightness, 0.5); // This was here orignally
        controls.set(controls::AE_ENABLE, true);

        requests.push_back(std::move(request));
    }
    // banana

    /*
     * --------------------------------------------------------------------
     * Signal&Slots
     *
     * libcamera uses a Signal&Slot based system to connect events to
     * callback operations meant to handle them, inspired by the QT graphic
     * toolkit.
     *
     * Signals are events 'emitted' by a class instance.
     * Slots are callbacks that can be 'connected' to a Signal.
     *
     * A Camera exposes Signals, to report the completion of a Request and
     * the completion of a Buffer part of a Request to support partial
     * Request completions.
     *
     * In order to receive the notification for request completions,
     * applications shall connecte a Slot to the Camera 'requestCompleted'
     * Signal before the camera is started.
     */
    camera->requestCompleted.connect(requestComplete);

    /*
     * --------------------------------------------------------------------
     * Start Capture
     *
     * In order to capture frames the Camera has to be started and
     * Request queued to it. Enough Request to fill the Camera pipeline
     * depth have to be queued before the Camera start delivering frames.
     *
     * For each delivered frame, the Slot connected to the
     * Camera::requestCompleted Signal is called.
     */

    // Should return 0 if camera is started
    if (camera->start())
    {
        std::cout << "Camera not started\n\n";
        return EXIT_FAILURE;
    }
    else
    {
        std::cout << "\n\n-------------------- [ CAMERA STARTED] --------------------\n\n";
    }
    for (std::unique_ptr<Request> &request : requests)
    {
        // Should return 0 if everything is okay
        if (camera->queueRequest(request.get()))
        {
            std::cout << "Queue request denied\n\n";
            return EXIT_FAILURE;
        }
        // std::cout << "Request : " << camera->queueRequest(request.get()) << std::endl;
    }
    // camera->queueRequest(request.get());
    /*
     * --------------------------------------------------------------------
     * Run an EventLoop
     *
     * In order to dispatch events received from the video devices, such
     * as buffer completions, an event loop has to be run.
     */

    // Have loop run indefinently
    loop.exec();

    // loop.timeout(TIMEOUT_SEC);
    // int ret = loop.exec();
    // std::cout << "Capture ran for " << TIMEOUT_SEC << " seconds and "
    //           << "stopped with exit status: " << ret << std::endl;

    /*
     * --------------------------------------------------------------------
     * Clean Up
     *
     * Stop the Camera, release resources and stop the CameraManager.
     * libcamera has now released all resources it owned.
     */
    camera->stop();
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
    cm->stop();

    return EXIT_SUCCESS;
}