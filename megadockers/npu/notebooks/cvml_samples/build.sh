#!/bin/bash
#
# Build script for CVML samples
#

set -e  # Exit on error

echo "Building CVML samples..."

# Create build directory
BUILD_DIR="build"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR

# Run cmake
echo "Running CMake..."
cmake -S $PWD -B $PWD/$BUILD_DIR

# Build
echo "Building executables..."
cmake --build $PWD/$BUILD_DIR --config Release

echo ""
echo "Build complete!"
echo ""
echo "Executables created:"
echo "  - $BUILD_DIR/depth_estimation_npu"
echo "  - $BUILD_DIR/depth_estimation_cpu"
echo "  - $BUILD_DIR/face_detection_npu"
echo "  - $BUILD_DIR/face_detection_cpu"
echo ""
echo "Usage examples:"
echo "  ./$BUILD_DIR/depth_estimation_npu -i ../Ryzen-AI-CVML-Library/samples/video_call.mp4 -o output_npu.mp4"
echo "  ./$BUILD_DIR/depth_estimation_cpu -i ../Ryzen-AI-CVML-Library/samples/video_call.mp4 -o output_cpu.mp4"
echo "  ./$BUILD_DIR/face_detection_npu -i ../Ryzen-AI-CVML-Library/samples/video_call.mp4 -m fast -o face_npu.mp4"
echo "  ./$BUILD_DIR/face_detection_cpu -i ../Ryzen-AI-CVML-Library/samples/video_call.mp4 -m fast -o face_cpu.mp4"
echo ""
