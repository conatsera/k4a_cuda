// k4a_cuda.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#ifndef _K4A_CUDA_H
#define _K4A_CUDA_H

#include "k4a/k4a.h"
#include "k4abt.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector_types.h>

#include <cstdio>

//#include <cstdio>

constexpr int kThreadXSize = 512;
constexpr int kThreadYSize = 512;

constexpr int kSize = kThreadXSize * kThreadYSize;

typedef struct joint {
	double3 position;
	double4 orientation;
} joint_t;

typedef struct skeleton {
	joint_t head;
	joint_t waist;
	joint_t right_wrist;
	joint_t left_wrist;
	joint_t right_ankle;
	joint_t left_ankle;
} skeleton_t;

typedef struct skeleton_group {
	int count;
	skeleton_t* skeletons;
} skeleton_group_t;

class K4A_CudaPointCloud {
public:
	K4A_CudaPointCloud();
	~K4A_CudaPointCloud();

	void GetCapture();

	float4* GeneratePointCloud();
	int GetSkeletonCount();
	float3* GetSkeletonJoints();
	float4* GetSkeletonJointsRots();
private:
	k4a_device_t device = NULL;

	k4a_calibration_t sensor_calibration;
	int dots;

	k4a_wait_result_t get_capture_result;
	k4a_capture_t capture = NULL;

	k4abt_tracker_t tracker = NULL;

	int skeleton_count;
	float4* joint_rots;

	float2* h_xy_table;
	float4* h_point_cloud;

	void CreateXYTable();
};

// TODO: Reference additional headers your program requires here.

#endif