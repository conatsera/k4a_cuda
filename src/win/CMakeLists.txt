add_library(k4a_cuda_win SHARED dllmain.cpp)

target_include_directories(k4a_cuda_win PUBLIC "${K4A_SDK_INCLUDE};${K4ABT_SDK_INCLUDE};${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")

target_link_libraries(k4a_cuda_win k4a_cuda)