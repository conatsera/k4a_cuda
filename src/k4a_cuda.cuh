// k4a_cuda.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#ifndef _K4A_CUDA_H
#define _K4A_CUDA_H

#include "k4a/k4a.h"
#include "k4abt.h"
#include "k4abttypes.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector_types.h>

#include <cstdio>

constexpr int kThreadXSize = 512;
constexpr int kThreadYSize = 512;

constexpr int kSize = kThreadXSize * kThreadYSize;

#define MAX_TRACKED_SKELETONS 4

typedef struct _k4a_skeleton_group_t
{
	k4abt_skeleton_t skeletons[MAX_TRACKED_SKELETONS];
} k4a_skeleton_group_t;

class K4A_CudaPointCloud {
public:
	K4A_CudaPointCloud();
	~K4A_CudaPointCloud();

	void GetCapture();

	float4* GeneratePointCloud();
	int GetSkeletonCount();
	void GetSkeletons();
	k4abt_skeleton_t GetSkeleton(int skel_id);
private:
	k4a_device_t device = NULL;

	k4a_calibration_t sensor_calibration;
	int dots;

	k4a_capture_t capture = NULL;
	k4abt_tracker_t tracker = NULL;

	float2* h_xy_table;
	float4* h_point_cloud;

	k4abt_frame_t body_frame = NULL;
	int skeleton_count;
	k4a_skeleton_group_t skeleton_group;

	void CreateXYTable();
};

// TODO: Reference additional headers your program requires here.

#endif