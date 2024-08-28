cmake_minimum_required(VERSION 3.0)
project(CameraCalibration)

# Set the C++ standard you are using (e.g., C++11, C++14, C++17)
set(CMAKE_CXX_STANDARD 11)

# Find OpenCV package in the system
find_package(OpenCV REQUIRED)

# Add your source file
set(SOURCES
    main.cpp
)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the executable for your application
#add_executable(camera_calibration camera_calibration.cpp)
add_executable(CameraCalibration ${SOURCES})

# Link your application with OpenCV libraries
target_link_libraries(CameraCalibration ${OpenCV_LIBS})
