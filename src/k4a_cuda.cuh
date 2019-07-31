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

// For cuda intellisense, recomment to build
//#define __CUDACC__
//#include <device_functions.h>
//#include <crt/math_functions.h>

#include <cstdio>

#define MAX_TRACKED_SKELETONS 4

#define MEMORY_ALIGNMENT  4096
#define ALIGN_UP(x,size) ( ((size_t)x+(size-1))&(~(size-1)) )

typedef struct _k4a_skeleton_group_t
{
	k4abt_skeleton_t skeletons[MAX_TRACKED_SKELETONS];
} k4a_skeleton_group_t;

class K4A_CudaPointCloud {
public:
	K4A_CudaPointCloud(bool color, bool body_tracking);
	~K4A_CudaPointCloud();

	void GetCapture();

	void SetupPointCloud();
	float4* GeneratePointCloud();
	uint32_t* GetPointColors();
	uint32_t GetPointCount();
	void ResetPointCloud();

	void SetSkeletonGroup(k4a_skeleton_group_t* group_ref);
	int GetSkeletonCount();
	void GetSkeletons();

#ifdef IS_TEST
	float4* GetSDKPointCloud();
	uint32_t* GetSDKPointColors();
#endif
private:
	k4a_device_t device = NULL;

	k4a_calibration_t sensor_calibration;
	int dots;
	int depth_points;
	bool color_enabled = false;

	k4a_capture_t capture = NULL;
	k4abt_tracker_t tracker = NULL;

	k4a_image_t depth_image;
	k4a_image_t color_image;

	float2* h_xy_table;
	float2* d_xy_table;

	float4* h_point_cloud;
	float4* d_point_cloud;
	float4* d_color_point_cloud;
	float4* d_trimmed_point_cloud;

	uint32_t* h_color_points;
	uint32_t* d_color_points;
	uint32_t* d_trimmed_color_points;

	unsigned int* d_point_count;
	unsigned int* h_point_count;

	uint16_t* h_depth_data;
	uint16_t* d_depth_data;

	uint32_t* h_color_data;
	uint32_t* d_color_data;

	int2* h_dimensions;
	int2* d_dimensions;

	k4abt_frame_t body_frame = NULL;
	int skeleton_count;

	k4a_skeleton_group_t* h_skeleton_group;
	k4a_skeleton_group_t* h_skeleton_group_unadjusted;
	k4a_skeleton_group_t* d_skeleton_group;
	k4a_skeleton_group_t* d_skeleton_group_unadjusted;

	void CreateXYTable();
};

// TODO: Reference additional headers your program requires here.

#endif