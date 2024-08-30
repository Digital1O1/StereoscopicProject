# Current issues
- [ ] When using ```cv::resize``` at `1920 x 1080` RPI will freeze 
  - Doubtful there's a work around due to the following reasons
    1) Working with very large images and using resize is computationally intenstive 
    2) Resizing consumes alot of memory and if there isn't enough the system will freeze 
    3) Using the wrong parameters like `an invalid interpolation method` will cause issues
  - Things you could try
    - Look into increasing the 'memory' to see if that solves the issue
    - Or just use a higher resolution and don't crop it
    - ~~Reduce the image size to downsample the image first by using a different interpolation method like `cv::INTER_AREA`~~
    - ~~Trying out different `interpolation methods` that could be faster like~~
      - ~~`cv::INTER_NEAREST`~~
        - ~~Typically the fastest~~
      - ~~`cv::INTER_CUBIC`~~
        - ~~Slower but provides better quality~~
  - ~~Attemps that were made~~
    - ~~Using `cv::INTER_NEAREST` for nearest neighbor interpolation~~
      - ~~`Still froze after 4 seconds`~~

---

# Things to look into
- Single Instruction Multiple Data NEON support for ARM/Raspberry Pi
  - Determine what kind of support is out there for the RPI
    - https://answers.opencv.org/question/33940/are-these-functions-accelerated-by-arm-neon/
  - Useful references?
  -   https://tttapa.github.io/Pages/Raspberry-Pi/NEON/index.html
  -   https://developer.arm.com/documentation/den0018/a/
  -   https://developer.arm.com/documentation/102467/0201/Example---matrix-multiplication
  -   https://github.com/PhotonVision/photon-libcamera-gl-driver
  -   Using the libcamera in C++ application
      -   https://libcamera.org/guides/application-developer.html

---

# Things that you learned along the way

## SMID

## cv::resize() can be used with float values 
- [Reference link](https://learnopencv.com/image-resizing-with-opencv/#resize-with-interpolation)
  ```cpp
  // Scaling Up the image 1.2 times by specifying both scaling factors
  double scale_up_x = 1.2;
  double scale_up_y = 1.2;
  // Scaling Down the image 0.6 times specifying a single scale factor.
  double scale_down = 0.6;
  Mat scaled_f_up, scaled_f_down;
  //resize 
  resize(image,scaled_f_down, Size(), scale_down, scale_down, INTER_LINEAR);
  resize(image, scaled_f_up, Size(), scale_up_x, scale_up_y, INTER_LINEAR);
  ```
## Interpolation
- What is it 
  - [Information from first reference](https://medium.com/@nuwanthidileka/image-interpolation-5e4cbe90603a)
  - The process of using known data points to estimate values at unknown locations
  - This works in `two different directions` and tries to achieve the best approxiamtion of a pixel's intensity based on the values of the `surrounding pixels`
  - It's a `approximation method` image that will `ALWAYS lose some quality` when interpolated
- Interpolation typically happens when an image is `resized` or `distorted/remapped` from one-pixel grid to another
- There's a ton of techniques that can be grouped into two categroies 
  - `Adaptive`
    - Changes depending on what they're interpolating; sharp versus smooth textures
    - Non-adpative methods treat app pixel values the same
  - `Non-adaptive`
- This article focuses mainly on non-adaptive methods like
    - `Nearest neighbor Interpolation`  
    - Most basic form of interpolation
    - This algorithm only consideres one pixel, the closest one to the interpolated point
    - Requires the least processing time of all the interpolation algorithms 
    - Has the effect of `making each pixel bigger`
  - `Bilinear Interpolation`
    - Consideres the closest 2x2 neighborhood of known pixel values
    -   Total of 4 that surround the unknown pixel
    -   Takes `average weight` of the values assigned to the unknown pixel
    -   Results in smoother-looking image in comparison to the `nearest neighbor` 
        -   Needs more processing time
  - `Bicubic Interpolation`
    - Considers the closest 4x4 neighborhood of known pixel values
      - Total of 16 that surround the unknown pixel
      - Closer pixels will have `higher weighting`
    - Produces noticeably sharper images than `nearest neighbor` and `bilinear interpolations`
    - Bicubic is the `ideal combination` of process time and output quality 

---

# [Stereo Camera Calibration and Triangulation with OpenCV and Python](https://temugeb.github.io/opencv/python/2021/02/02/stereo-camera-calibration-and-triangulation.html)

## Big picture of article 
- Calibrate each camera seperately using checkerboard 
- Calibrate both cameras
- Use direct linear transform (DLT) to triangulate camera pixels to 3d coordinates

---

# Image Misalignment Troubleshooting Checklist

## 1. Verify Camera Parameters
- [ ] **Camera Matrix and Distortion Coefficients:** Ensure that the camera matrices and distortion coefficients for both cameras are accurate.
- [ ] **Rotation and Translation Vectors:** Check the rotation and translation vectors between the cameras to ensure correct relative positioning.

## 2. Check Calibration Quality
- [ ] **Reprojection Error:** Verify the reprojection error after calibration. A high reprojection error suggests inaccurate calibration.
- [ ] **Calibration Process:** Confirm that the calibration process was done correctly with well-distributed and sufficient calibration images.

## 3. Image Rectification
- [ ] **Rectify Images:** Apply the rectification process to align the images so that corresponding points appear on the same horizontal line.
- [ ] **Check Rectification Output:** Visualize the rectified images to ensure proper alignment.

## 4. Refine the Stereo Calibration
- [ ] **Manual Adjustment:** Consider manually fine-tuning the calibration parameters to improve alignment.
- [ ] **Recalibrate:** If necessary, recalibrate the cameras using a sufficient number of high-quality calibration images.

## 5. Consider Camera Setup
- [ ] **Camera Mounting:** Ensure that the cameras are mounted rigidly and parallel to each other to avoid physical misalignment.
- [ ] **Synchronization:** Check if the cameras are synchronized to prevent time lag between captured frames.

## 6. Test with Known Patterns
- [ ] **Use a Checkerboard:** Test the alignment using a known pattern like a checkerboard. Overlay the rectified images and verify the alignment.
- [ ] **Epipolar Lines:** Draw epipolar lines on the images. Corresponding points should lie on the same epipolar line for well-calibrated images.

## 7. Re-evaluate the Image Processing Pipeline
- [ ] **Pipeline Consistency:** Ensure the same distortion correction and rectification process is applied to both images before overlaying them.
- [ ] **Image Scaling:** Verify that no unintended scaling or transformation is applied to one of the images, causing misalignment.

---

## Stereoscopic Calibration Checklist for IMX219 Cameras

- [x] **Calculate Camera Matrix and Distortion Coefficients**
  - Ensure both cameras' matrices and coefficients are calculated.
  - Verify the ability to take pictures from both cameras.

- [x] **Stereo Calibration**
  - [x] **Capture Stereo Image Pairs**
    - Capture multiple images of a checkerboard from both cameras simultaneously at different angles and positions.
  - [x] **Find Chessboard Corners**
    - Use `cv::findChessboardCorners` to detect the corners of the checkerboard in each image pair.
  - [x] **Run Stereo Calibration**
    - Use `cv::stereoCalibrate` to compute the relative position and orientation of the two cameras (obtain `R`, `T`, `E`, `F`).

- [ ] **Rectification**
  - [ ] **Stereo Rectification**
    - Use `cv::stereoRectify` to compute the rectification transforms (`R1`, `R2`, `P1`, `P2`, `Q`) for both cameras.
  - [ ] **Remapping**
    - Generate rectification maps using `cv::initUndistortRectifyMap`.
    - Apply the rectification maps to the stereo image pairs using `cv::remap`.

- [ ] **Disparity Map Computation**
  - [ ] Use `cv::StereoBM` or `cv::StereoSGBM` to compute the disparity map from the rectified stereo image pairs.

- [ ] **Depth Map Calculation**
  - [ ] Convert the disparity map to a depth map using the `Q` matrix obtained from `cv::stereoRectify`.
  - [ ] Use `cv::reprojectImageTo3D` to map the disparity map into a 3D space, creating the depth map.

- [ ] **Testing and Refinement**
  - [ ] Test the calibration by capturing new stereo image pairs.
  - [ ] Check the accuracy of the depth map.
  - [ ] Fine-tune parameters, especially for disparity computation, to improve depth estimation.
