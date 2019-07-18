// k4a_cuda.cpp : Defines the entry point for the application.
//

#include "k4a_cuda.cuh"

__global__ void k4a_point_cloud_adjust(uint16_t* depth_data, float2* xy_table, float4* point_cloud)
{
	int i = (blockIdx.x * 512) + (threadIdx.x);

	float this_depth_data = __uint2float_rn(depth_data[i]);

	point_cloud[i].x = __fdividef((this_depth_data * xy_table[i].x), 1000.0f);
	point_cloud[i].y = __fdividef((this_depth_data * xy_table[i].y), 1000.0f);
	point_cloud[i].z = __fdividef(this_depth_data, 1000.0f);

}

void K4ALogger(void* ctx, k4a_log_level_t logLevel, const char* file, const int line, const char* message)
{
	printf("%d %s:%d %s", logLevel, file, line, message);
}

void ConstantInitFloat(float* data, int size, float val)
{
	for (int i = 0; i < size; ++i)
	{
		data[i] = val;
	}
}

void ConstantInitFloat2(float2* data, int size, float2 val)
{
	for (int i = 0; i < size; ++i)
	{
		data[i] = val;
	}
}

void ConstantInitFloat4(float4* data, int size, float4 val)
{
	for (int i = 0; i < size; ++i)
	{
		data[i] = val;
	}
}

K4A_CudaPointCloud::K4A_CudaPointCloud()
{
	bool success = true;

	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;

	if (k4a_device_open(K4A_DEVICE_DEFAULT, &device) != K4A_RESULT_SUCCEEDED)
	{
		printf("Open K4A Device failed\n");
		success = false;
	}

	if (k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration) != K4A_RESULT_SUCCEEDED)
	{
		printf("Get depth camera calibration failed!\n");
		success = false;
	}
	else
	{
		dots = sensor_calibration.depth_camera_calibration.resolution_height * sensor_calibration.depth_camera_calibration.resolution_width;
		h_xy_table = new float2[dots];
	}

	if (success)
	{
		CreateXYTable();

		if (k4a_device_start_cameras(device, &device_config) != K4A_RESULT_SUCCEEDED)
		{
			printf("Start K4A cameras failed!\n");
		}

		if (k4abt_tracker_create(&sensor_calibration, &tracker) != K4A_RESULT_SUCCEEDED)
		{
			printf("Start tracker failed!\n");
		}
	}
}

K4A_CudaPointCloud::~K4A_CudaPointCloud()
{
	k4a_capture_release(capture);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
	delete h_xy_table;
	delete h_point_cloud;
}

void K4A_CudaPointCloud::CreateXYTable()
{
	h_xy_table = (float2*)malloc(kSize * sizeof(float2));
	ConstantInitFloat2(h_xy_table, kSize, make_float2(nanf(""), nanf("")));

	int width = sensor_calibration.depth_camera_calibration.resolution_width;
	int height = sensor_calibration.depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid = 0;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				&sensor_calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				h_xy_table[idx].x = ray.xyz.x;
				h_xy_table[idx].y = ray.xyz.y;
			}
		}
	}
}

void K4A_CudaPointCloud::GetCapture()
{
	k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
}

float4* K4A_CudaPointCloud::GeneratePointCloud()
{
	// Probe for a depth16 image
	k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
	if (depth_image == nullptr)
	{
		printf("Failed to get depth image from capture\n");
	}

	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);

	float* h_depth_data = (float*)malloc(kSize * sizeof(float));

	h_point_cloud = (float4*)malloc(kSize * sizeof(float4));

	ConstantInitFloat4(h_point_cloud, kSize, make_float4(nanf(""), nanf(""), nanf(""), nanf("")));

	uint16_t* d_depth_data;
	cudaMalloc(&d_depth_data, kSize * sizeof(uint16_t));
	float2* d_xy_table;
	cudaMalloc(&d_xy_table, kSize * sizeof(float2));
	float4* d_point_cloud;
	cudaMalloc(&d_point_cloud, kSize * sizeof(float4));

	cudaMemcpy(d_depth_data, depth_data, kSize * sizeof(uint16_t), cudaMemcpyHostToDevice);
	cudaMemcpy(d_xy_table, h_xy_table, kSize * sizeof(float2), cudaMemcpyHostToDevice);

	k4a_point_cloud_adjust << <512, 512 >> > (d_depth_data, d_xy_table, d_point_cloud);
	cudaDeviceSynchronize();

	cudaMemcpy(h_point_cloud, d_point_cloud, kSize * sizeof(float4), cudaMemcpyDeviceToHost);

	cudaFree(d_depth_data);
	cudaFree(d_xy_table);
	cudaFree(d_point_cloud);

	k4a_image_release(depth_image);

	return h_point_cloud;
}

int K4A_CudaPointCloud::GetSkeletonCount()
{
	return skeleton_count;
}

void K4A_CudaPointCloud::GetSkeletons()
{
	skeleton_count = 0;

	k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
	//k4a_capture_release(sensor_capture);
	if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		// It should never hit timeout when K4A_WAIT_INFINITE is set.
		printf("Error! Add capture to tracker process queue timeout!\n");
	}
	else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
	{
		printf("Error! Add capture to tracker process queue failed!\n");
	}

	k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
		printf("%zu bodies are detected!\n", num_bodies);

		for (int i = 0; i < num_bodies && i < MAX_TRACKED_SKELETONS; i++)
		{
			k4a_result_t get_body_result = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton_group.skeletons[skeleton_count]);
			if (get_body_result == K4A_RESULT_SUCCEEDED)
			{
				skeleton_count++;
			}
		}

		for (int skel_id = skeleton_count; skel_id < MAX_TRACKED_SKELETONS; skel_id++) {
			skeleton_group.skeletons[skel_id] = k4abt_skeleton_t();
		}

		k4abt_frame_release(body_frame);
	}
	else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		//  It should never hit timeout when K4A_WAIT_INFINITE is set.
		printf("Error! Pop body frame result timeout!\n");
	}
	else
	{
		printf("Pop body frame result failed!\n");
	}
}

k4abt_skeleton_t K4A_CudaPointCloud::GetSkeleton(int skel_id)
{
	return skeleton_group.skeletons[skel_id];
}