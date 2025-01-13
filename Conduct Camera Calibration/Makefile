# Compiler and flags
CXX = g++
CXXFLAGS = -Wall  -std=c++17

# Include directories for OpenCV
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS = $(shell pkg-config --libs opencv4)

# Output binary
TARGET = camera_pipeline

# Source files
SRC = main.cpp

# Build rules if multiple targets were present/needed which in this case is NOT
# all: $(TARGET)

# $@ Represents the target name used to name the compiled program
# $^ Represents the list of dependencies (the libraries and include paths) for the current target. Is a list of files needed to build the current target
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $^ $(OPENCV_LIBS) 

# Clean up build files
clean:
	rm -f $(TARGET)