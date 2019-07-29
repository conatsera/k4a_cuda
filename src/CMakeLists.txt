# CMakeList.txt : CMake project for k4a_cuda, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.14.0)

# Add source to this project's executable.
add_library (k4a_cuda STATIC "k4a_cuda.cu" "k4a_cuda.cuh")

target_include_directories(k4a_cuda PRIVATE "${K4A_SDK_INCLUDE};${K4ABT_SDK_INCLUDE};${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")

target_link_libraries(k4a_cuda ${K4A_SDK} ${K4ABT_SDK})

if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
	add_subdirectory(win)
elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	add_subdirectory(linux)
else()
    message(FATAL_ERROR "Unknown system: ${CMAKE_SYSTEM_NAME}")
endif()

file(COPY ${K4ABT_DNN}
	DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# TODO: Add tests and install targets if needed.
