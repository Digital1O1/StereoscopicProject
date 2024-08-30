# Variables
CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -Winvalid-pch -Wnon-virtual-dtor -Wextra  -Wno-unused-parameter

PKG_CONFIG = pkg-config

LIBCAMERA_CFLAGS = $(shell $(PKG_CONFIG) --cflags libcamera)
LIBCAMERA_LIBS = $(shell $(PKG_CONFIG) --libs libcamera)

LIBEVENT_CFLAGS = $(shell $(PKG_CONFIG) --cflags libevent_pthreads)
LIBEVENT_LIBS = $(shell $(PKG_CONFIG) --libs libevent_pthreads)

OPENCV_CFLAGS = $(shell $(PKG_CONFIG) --cflags opencv4)
OPENCV_LIBS = $(shell $(PKG_CONFIG) --libs opencv4)

# Include directories
# The '.' tells the compiler to include the current directory in the list of directories to search for the header files
INCLUDES = -I. $(LIBCAMERA_CFLAGS) $(LIBEVENT_CFLAGS) $(OPENCV_CFLAGS) \
           -I/home/pi/libcamera/include/linux \
           -I/usr/include \
           -I/home/pi/yaml-cpp/include

# Libraries
LIBS = $(LIBCAMERA_LIBS) $(LIBEVENT_LIBS) $(OPENCV_LIBS)

# Source files
COMMON_SRCS = event_loop.cpp image.cpp
SRCS = calibrationDualCamera.cpp $(COMMON_SRCS)

# Output executables
TARGET = DualCameraCalibration

# Default target
all: $(TARGET) 

# Link object files to create executables
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)
# $@ = name of target file
# $  = represents all prerequisites/dependencies of target


# Clean up
clean:
	rm -f $(TARGET)

.PHONY: all clean
