# [Stereo Camera Calibration and Triangulation with OpenCV and Python](https://temugeb.github.io/opencv/python/2021/02/02/stereo-camera-calibration-and-triangulation.html)

## Big picture of article 
- Calibrate each camera seperately using checkerboard 
- Calibrate both cameras
- Use direct linear transform (DLT) to triangulate camera pixels to 3d coordinates

## Stereoscopic Calibration Checklist for IMX219 Cameras

- [x] **Calculate Camera Matrix and Distortion Coefficients**
  - Ensure both cameras' matrices and coefficients are calculated.
  - Verify the ability to take pictures from both cameras.

- [ ] **Stereo Calibration**
  - [x] **Capture Stereo Image Pairs**
    - Capture multiple images of a checkerboard from both cameras simultaneously at different angles and positions.
  - [x] **Find Chessboard Corners**
    - Use `cv::findChessboardCorners` to detect the corners of the checkerboard in each image pair.
  - [ ] **Run Stereo Calibration**
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
