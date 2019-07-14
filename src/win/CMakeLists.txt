add_library(k4a_cuda_dll SHARED dllmain.cpp)

target_include_directories(k4a_cuda_dll PUBLIC "${PROJECT_INCLUDE_DIR};${K4A_SDK_INCLUDE};${K4ABT_SDK_INCLUDE};${CUDA_INCLUDE}")

target_link_libraries(k4a_cuda_dll k4a_cuda)