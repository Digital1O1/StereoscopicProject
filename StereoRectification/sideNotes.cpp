                if (!calibrationComplete)
                {
                    // The findChessboardCorners() is computationally heavy and will cause a noticeable visual delay if not inside this `if` statement
                    bool foundLeft = findChessboardCorners(leftGray, CHESSBOARD_SIZE, cornersLeft, chessBoardFlags);
                    bool foundRight = findChessboardCorners(leftGray, CHESSBOARD_SIZE, cornersRight, chessBoardFlags);

                    if (foundLeft && foundRight)
                    {
                        cv::cornerSubPix(leftGray, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
                        cv::drawChessboardCorners(leftGray, CHESSBOARD_SIZE, cornersLeft, foundLeft);

                        cv::cornerSubPix(rightGray, cornersRight, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
                        cv::drawChessboardCorners(rightGray, CHESSBOARD_SIZE, cornersRight, foundRight);

                        imagePointsLeft.push_back(cornersLeft);
                        objectPointsLeft.push_back(objectPointsListLeft);

                        imagePointsLeft.push_back(cornersRight);
                        objectPointsLeft.push_back(objectPointsListRight);

                        imagesCaptured++;

                        std::cout << "\nNumber of images captured " << ": [ " << imagesCaptured << " ] out of [ " << NUMBER_OF_IMAGES << " ] \n ";
                        cv::waitKey(100);
                    }


                    if (imagesCaptured == NUMBER_OF_IMAGES)
                    {
                        cv::destroyWindow("PREVIEW WINDOW");
                        std::cout << "\n ---------------------------------------------------------------------------" << std::endl;

                        printf("\nGenerating camera calibration values and distortion values\r\n");

                        cv::calibrateCamera(objectPointsLeft, imagePointsLeft, leftCameraRGB.size(), cameraMatrixLeft, distCoeffsLeft, Rl, Tl);

                        std::cout << std::endl
                                  << "Left Camera Matrix:\n\n"
                                  << cameraMatrixLeft << std::endl;
                        std::cout << std::endl
                                  << "Left Distortion Coefficients:\n\n"
                                  << distCoeffsLeft << std::endl;

                        std::cout << std::endl
                                  << "Right Camera Matrix:\n\n"
                                  << cameraMatrixRight << std::endl;
                        std::cout << std::endl
                                  << "Right Distortion Coefficients:\n\n"
                                  << distCoeffsRight << std::endl;

                        leftCameraYAMLFS << "cameraMatrixLeft" << cameraMatrixLeft;
                        leftCameraYAMLFS << "distortionCoefficients" << distCoeffsLeft;

                        rightCameraYAMLFS << "cameraMatrixLeftRight" << cameraMatrixRight;
                        rightCameraYAMLFS << "distortionCoefficientsRight" << distCoeffsRight;

                        leftCameraYAMLFS.release();
                        rightCameraYAMLFS.release();

                        std::cout << "Camera Matrix YAML values : " << cameraMatrixLeft << std::endl;
                        std::cout << "Distortion Coefficients YAML values : " << distCoeffsLeft << std::endl;

                        std::cout << "Camera Matrix YAML values : " << cameraMatrixRight << std::endl;
                        std::cout << "Distortion Coefficients YAML values : " << distCoeffsRight << std::endl;

                        std::cout << "Homography matricies are saved to : " << leftCameraYAML << " " << rightCameraYAML << std::endl;

                        calibrationComplete = true;
                        // std::cout << "Calibration status: " << calibrationComplete << std::endl;

                        printf("Calibration complete! Applying calculated camera matrix and distortion \r\n");
                    }

                    if (!calibrationComplete)
                    {
                        printf("Image not captured. Please reposition calibration chessboard.\r\n");

                        cv::Mat concatenated_image = concatenateImages(leftCameraRGB, rightCameraRGB);
                        displayImage(concatenated_image, "PREVIEW WINDOW");
                        cv::imshow("PREVIEW WINDOW", concatenated_image);
                        cv::waitKey(1);
                    }
                } // if (!calibrationComplete)