# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.14.0)

project ("k4a_cuda" LANGUAGES CXX CUDA VERSION 0.4)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# For code, link against the static crt on windows
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /ZH:SHA_256")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /ZH:SHA_256")
endif()

# If using clang or GCC only linked shared libraries if needed
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
endif()

# Find all dependencies
add_subdirectory(extern)

# Don't enable testing until after building dependencies
enable_testing()

# Get K4A SDKs
find_library(K4A_SDK k4a "C:/Program Files/Azure Kinect SDK v1.1.0/sdk/windows-desktop/amd64/release/lib")
find_library(K4ABT_SDK k4abt "C:/Program Files/Azure Kinect Body Tracking SDK/sdk/windows-desktop/amd64/release/lib")
find_file(K4ABT_DNN "dnn_model.onnx" "C:/Program Files/Azure Kinect Body Tracking SDK/sdk/windows-desktop/amd64/release/bin" NO_DEFAULT_PATH)

set(K4A_SDK_INCLUDE "C:/Program Files/Azure Kinect SDK v1.1.0/sdk/include")
set(K4ABT_SDK_INCLUDE "C:/Program Files/Azure Kinect Body Tracking SDK/sdk/include")
set(CUDA_INCLUDE "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1/include")

include_directories("src")

# Include sub-projects.
add_subdirectory ("src")
add_subdirectory("tests")

# CMake doesn't set the target processor correctly for MSVC
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    if ("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(TARGET_ARCH "amd64")
    elseif("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
        set(TARGET_ARCH "x86")
    else()
        message(FATAL_ERROR "Unknown architecture with size of void* = ${CMAKE_SIZEOF_VOID_P}")
    endif()
endif()