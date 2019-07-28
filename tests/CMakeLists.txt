include(GoogleTest)

add_executable(k4a_cuda_ut k4a_cuda_ut.cpp "${CMAKE_SOURCE_DIR}/src/k4a_cuda.cu" "${CMAKE_SOURCE_DIR}/src/k4a_cuda.cuh")

target_compile_definitions(k4a_cuda_ut PRIVATE IS_TEST)

target_link_libraries(k4a_cuda_ut PRIVATE
	gtest::gtest
    gtest::gmock
	${K4A_SDK}
	${K4ABT_SDK})

target_include_directories(k4a_cuda_ut PRIVATE "${gtest_SOURCE_DIR}/include;${gmock_SOURCE_DIR}/include;${K4A_SDK_INCLUDE};${K4ABT_SDK_INCLUDE};${CUDA_INCLUDE}")


gtest_discover_tests(k4a_cuda_ut)

add_executable(k4a_cuda_dll_ut k4a_cuda_dll_ut.cpp)

target_compile_definitions(k4a_cuda_dll_ut PRIVATE IS_TEST)

target_link_libraries(k4a_cuda_dll_ut PRIVATE
	k4a_cuda_win
	gtest::gtest
    gtest::gmock)

target_include_directories(k4a_cuda_dll_ut PRIVATE "${gtest_SOURCE_DIR}/include;${gmock_SOURCE_DIR}/include")


gtest_discover_tests(k4a_cuda_dll_ut)