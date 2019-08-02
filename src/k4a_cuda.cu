// k4a_cuda.cpp : Defines the entry point for the application.
//

#include "k4a_cuda.cuh"

__constant__ float calibration_3d_to_3c[12];
__constant__ float calibration_3c_to_2c[15];

__forceinline__ __device__ void compute_correspondence(float4* depth_point,
									   uint32_t* color_data,
									   float4* color_depth_point,
									   uint32_t* color_pixel,
									   int2* dimensions,
									   int point)
{
	const float x = depth_point[point].x;
	const float y = depth_point[point].y;
	const float z = depth_point[point].z;

	float4 color_point;
	color_point.x = calibration_3d_to_3c[0] * x + calibration_3d_to_3c[1] * y + calibration_3d_to_3c[2] * z + calibration_3d_to_3c[9];
	color_point.y = calibration_3d_to_3c[3] * x + calibration_3d_to_3c[4] * y + calibration_3d_to_3c[5] * z + calibration_3d_to_3c[10];
	color_point.z = calibration_3d_to_3c[6] * x + calibration_3d_to_3c[7] * y + calibration_3d_to_3c[8] * z + calibration_3d_to_3c[11];

	color_depth_point[point].x = __fdividef(color_point.x, 1000.0f);
	color_depth_point[point].y = __fdividef(color_point.y, -1000.0f);
	color_depth_point[point].z = __fdividef(color_point.z, 1000.0f);

	float xy[2];
	xy[0] = color_point.x / color_point.z;
	xy[1] = color_point.y / color_point.z;

	float xp = xy[0] - calibration_3c_to_2c[10];
	float yp = xy[1] - calibration_3c_to_2c[11];

	float xp2 = xp * xp;
	float yp2 = yp * yp;
	float xyp = xp * yp;
	float rs = xp2 + yp2;

	if (!(rs > calibration_3c_to_2c[14] * calibration_3c_to_2c[14]))
	{
		float rss = rs * rs;
		float rsc = rss * rs;
		float a = 1.f + calibration_3c_to_2c[4] * rs + calibration_3c_to_2c[5] * rss + calibration_3c_to_2c[6] * rsc;
		float b = 1.f + calibration_3c_to_2c[7] * rs + calibration_3c_to_2c[8] * rss + calibration_3c_to_2c[9] * rsc;
		float bi;
		if (b != 0.f)
		{
			bi = 1.f / b;
		}
		else
		{
			bi = 1.f;
		}
		float d = a * bi;

		float xp_d = xp * d;
		float yp_d = yp * d;

		float rs_2xp2 = rs + 2.f * xp2;
		float rs_2yp2 = rs + 2.f * yp2;

		xp_d += rs_2xp2 * calibration_3c_to_2c[13] + 2.f * xyp * calibration_3c_to_2c[12];
		yp_d += rs_2yp2 * calibration_3c_to_2c[12] + 2.f * xyp * calibration_3c_to_2c[13];

		float xp_d_cx = xp_d + calibration_3c_to_2c[10];
		float yp_d_cy = yp_d + calibration_3c_to_2c[11];

		float2 color_pixel_xy;
		color_pixel_xy.x = xp_d_cx * calibration_3c_to_2c[2] + calibration_3c_to_2c[0];
		color_pixel_xy.y = yp_d_cy * calibration_3c_to_2c[3] + calibration_3c_to_2c[1];

		int pixel = (__float2int_rz(color_pixel_xy.y) * 2048) + __float2int_rz(color_pixel_xy.x);

		color_pixel[point] = color_data[pixel];
		color_depth_point[point].w = 1.0f;
	}
}

__global__ void transform_to_color_cloud(float4* point_cloud,
									uint32_t* color_data,
									float4* color_point_cloud,
									uint32_t* point_colors,
									int2* dimensions)
{
	int pixel_ratio = dimensions->x / blockDim.x;
	for (int j = 0; j < pixel_ratio; j++)
	{
		int pixel = (blockIdx.x * blockDim.x * j) + (threadIdx.x);
		compute_correspondence(point_cloud, color_data, color_point_cloud, point_colors, dimensions, pixel);
	}
}

__global__ void k4a_point_cloud_adjust(uint16_t* depth_data, float2* xy_table, float4* point_cloud)
{
	int i = (blockIdx.x * blockDim.x) + (threadIdx.x);

	float this_depth_data = __uint2float_rz(depth_data[i]);

	if (this_depth_data != nanf("") && this_depth_data != 0.0f)
	{
		point_cloud[i].x = __fdividef((this_depth_data * xy_table[i].x), 1000.0f);
		point_cloud[i].y = __fdividef((this_depth_data * xy_table[i].y), -1000.0f);
		point_cloud[i].z = __fdividef(this_depth_data, 1000.0f);
	}
}

__global__ void k4a_color_point_cloud_adjust(uint16_t* depth_data, float2* xy_table, float4* point_cloud)
{
	int i = (blockIdx.x * blockDim.x) + (threadIdx.x);

	float this_depth_data = __uint2float_rz(depth_data[i]);

	if (this_depth_data != nanf("") && this_depth_data != 0.0f)
	{
		point_cloud[i].x = (this_depth_data * xy_table[i].x);
		point_cloud[i].y = (this_depth_data * xy_table[i].y);
		point_cloud[i].z = this_depth_data;
	}
}

//__device__ uint32_t count = 0;
//__device__ bool is_last_thread_done;

__global__ void trim_and_conform_color(float4* color_point_cloud, float4* trimmed_cloud, uint32_t* color_points, uint32_t* trimmed_color_points, unsigned int* point_count)
{
		int pixel = (blockIdx.x * blockDim.x) + (threadIdx.x);
		if (color_point_cloud[pixel].w == 1.0f && color_point_cloud[pixel].z != 0.0f && color_point_cloud[pixel].x != nanf(""))
		{
			unsigned int point = atomicAdd(point_count, 1);

			trimmed_cloud[point] = color_point_cloud[pixel];
			trimmed_color_points[point] = color_points[pixel];
		}
}

__global__ void trim_and_conform(float4* point_cloud, float4* trimmed_cloud, unsigned int* point_count)
{
	int pixel = (blockIdx.x * blockDim.x) + (threadIdx.x);
	if (point_cloud[pixel].w == 1.0f && point_cloud[pixel].z != 0.0f && point_cloud[pixel].x != nanf(""))
	{
		unsigned int point = atomicAdd(point_count, 1);

		trimmed_cloud[point] = point_cloud[pixel];
	}
}

__global__ void k4a_skeleton_adjust(int skeleton_count, k4a_skeleton_group_t* skeletons, k4a_skeleton_group_t* skeletons_adjusted)
{
	for (int skel_id = 0; skel_id < skeleton_count; skel_id++)
	{
		int i = (blockIdx.x * 7) + (threadIdx.x);

		if (i < 27)
		{
			skeletons_adjusted->skeletons[skel_id].joints[i].orientation.wxyz.w = skeletons->skeletons[skel_id].joints[i].orientation.wxyz.w;
			skeletons_adjusted->skeletons[skel_id].joints[i].orientation.wxyz.x = skeletons->skeletons[skel_id].joints[i].orientation.wxyz.x;
			skeletons_adjusted->skeletons[skel_id].joints[i].orientation.wxyz.x = skeletons->skeletons[skel_id].joints[i].orientation.wxyz.y;
			skeletons_adjusted->skeletons[skel_id].joints[i].orientation.wxyz.z = skeletons->skeletons[skel_id].joints[i].orientation.wxyz.z;

			skeletons_adjusted->skeletons[skel_id].joints[i].position.xyz.x = __fdividef(skeletons->skeletons[skel_id].joints[i].position.xyz.x, -1000.0F);
			skeletons_adjusted->skeletons[skel_id].joints[i].position.xyz.y = __fdividef(skeletons->skeletons[skel_id].joints[i].position.xyz.y, -1000.0F);
			skeletons_adjusted->skeletons[skel_id].joints[i].position.xyz.z = __fdividef(skeletons->skeletons[skel_id].joints[i].position.xyz.z, 1000.0F);
		}
	}
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

K4A_CudaPointCloud::K4A_CudaPointCloud(bool color, bool body_tracking)
{
	bool success = true;
	color_enabled = color;

	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	if (color_enabled)
	{
		device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		device_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
		device_config.synchronized_images_only = true;
	}

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
		if (color_enabled)
			dots = sensor_calibration.color_camera_calibration.resolution_height* sensor_calibration.color_camera_calibration.resolution_width;
		else
			dots = sensor_calibration.depth_camera_calibration.resolution_height * sensor_calibration.depth_camera_calibration.resolution_width;
		depth_points = sensor_calibration.depth_camera_calibration.resolution_height * sensor_calibration.depth_camera_calibration.resolution_width;
	}

	if (success)
	{
		CUdevice cuda_device;
		cudaGetDevice(&cuda_device);

		unsigned int cuda_flags = CUctx_flags::CU_CTX_SCHED_BLOCKING_SYNC + CUctx_flags::CU_CTX_MAP_HOST;

		cuDevicePrimaryCtxSetFlags(cuda_device, cuda_flags);

		CreateXYTable();

		if (k4a_device_start_cameras(device, &device_config) != K4A_RESULT_SUCCEEDED)
		{
			printf("Start K4A cameras failed!\n");
		}

		if (body_tracking)
		{
			if (k4abt_tracker_create(&sensor_calibration, &tracker) != K4A_RESULT_SUCCEEDED)
			{
				printf("Start tracker failed!\n");
			}
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
	h_xy_table = (float2*)malloc(depth_points * sizeof(float2));
	ConstantInitFloat2(h_xy_table, depth_points, make_float2(nanf(""), nanf("")));

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
	if (capture != NULL)
		k4a_capture_release(capture);
	k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
}

float h_calibration_3d_to_3c[12];
float h_calibration_3c_to_2c[15];

int K4A_CudaPointCloud::GetMaxPointCount()
{
	return dots;
}

void K4A_CudaPointCloud::SetupPointCloud(float4** point_cloud, uint32_t** color_points)
{
	cudaMalloc(&d_depth_data, depth_points * sizeof(uint16_t));
	cudaMalloc(&d_xy_table, depth_points * sizeof(float2));
	cudaMalloc(&d_point_cloud, dots * sizeof(float4));

	//cudaMalloc(&d_trimmed_color_points, dots * sizeof(uint32_t));
	//cudaMalloc(&d_trimmed_point_cloud, dots * sizeof(float4));
	cudaMalloc(&d_point_count, sizeof(unsigned int));

	cudaMemcpy(d_xy_table, h_xy_table, depth_points * sizeof(float2), cudaMemcpyHostToDevice);

	h_point_count = (unsigned int*)malloc(sizeof(unsigned int));
	//h_point_cloud = (float4*)malloc(dots * sizeof(float4));
	h_point_cloud = (*point_cloud);
	empty_cloud = (float4*)malloc(dots * sizeof(float4));
	ConstantInitFloat4(empty_cloud, dots, make_float4(nanf(""), nanf(""), nanf(""), nanf("")));
	memcpy(h_point_cloud, empty_cloud, dots * sizeof(float4));

	cudaHostRegister(h_point_cloud, sizeof(float4) * dots, cudaHostRegisterMapped);

	cudaHostGetDevicePointer<float4>(&d_trimmed_point_cloud, h_point_cloud, 0);

	if (color_enabled)
	{
		int width = sensor_calibration.color_camera_calibration.resolution_width;
		int height = sensor_calibration.color_camera_calibration.resolution_height;

		//h_color_points = (uint32_t*)malloc(dots * sizeof(uint32_t));
		h_color_points = (*color_points);
		h_dimensions = new int2(make_int2(width, height));

		cudaHostRegister(h_color_points, sizeof(uint32_t) * dots, cudaHostRegisterMapped);

		cudaHostGetDevicePointer<uint32_t>(&d_trimmed_color_points, h_color_points, 0);

		cudaMalloc(&d_color_data, dots * sizeof(uint32_t));
		cudaMalloc(&d_color_points, dots * sizeof(uint32_t));
		cudaMalloc(&d_color_point_cloud, dots * sizeof(float4));
		cudaMalloc(&d_dimensions, sizeof(int2));

		cudaMemcpy(d_dimensions, h_dimensions, sizeof(int2), cudaMemcpyHostToDevice);

		for (int i = 0; i < 9; i++)
			h_calibration_3d_to_3c[i] = sensor_calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[i];
		for (int i = 0; i < 3; i++)
			h_calibration_3d_to_3c[i+9] = sensor_calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[i];
		
		for (int i = 0; i < 14; i++)
			h_calibration_3c_to_2c[i] = sensor_calibration.color_camera_calibration.intrinsics.parameters.v[i];
		h_calibration_3c_to_2c[14] = sensor_calibration.color_camera_calibration.metric_radius;

		cudaMemcpyToSymbol(calibration_3d_to_3c, &h_calibration_3d_to_3c, sizeof(float) * 12);
		cudaMemcpyToSymbol(calibration_3c_to_2c, &h_calibration_3c_to_2c, sizeof(float) * 15);
	}
	else
	{
		h_point_cloud = (float4*)malloc(depth_points * sizeof(float4));
	}
}

void K4A_CudaPointCloud::GeneratePointCloud()
{
	// Probe for a depth16 image
	depth_image = k4a_capture_get_depth_image(capture);
	if (depth_image == nullptr)
	{
		printf("Failed to get depth image from capture\n");
	}

	if (color_enabled)
	{
		memset(h_color_points, '\0', dots * sizeof(uint32_t));

		// Probe for a depth16 image
		color_image = k4a_capture_get_color_image(capture);
		if (color_image == nullptr)
		{
			printf("Failed to get color image from capture\n");
		}
	}

	h_depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);

	(*h_point_count) = 0;

	memcpy(h_point_cloud, empty_cloud, dots * sizeof(float4));

	//cudaMemset(d_depth_data, 0, depth_points * sizeof(uint16_t));

	cudaMemset(d_point_cloud, 0, dots * sizeof(float4));

	cudaMemcpy(d_depth_data, h_depth_data, depth_points * sizeof(uint16_t), cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();

	int blockSize = depth_points / 512;
	if (color_enabled)
		k4a_color_point_cloud_adjust << <blockSize, 512 >> > (d_depth_data, d_xy_table, d_point_cloud);
	else
		k4a_point_cloud_adjust << <blockSize, 512 >> > (d_depth_data, d_xy_table, d_point_cloud);

	if (color_enabled)
	{
		h_color_data = (uint32_t*)(void*)k4a_image_get_buffer(color_image);

		//cudaMemset(d_color_data, 0, dots * sizeof(uint32_t));

		cudaMemset(d_color_points, 0, dots * sizeof(uint32_t));

		cudaMemset(d_color_point_cloud, 0, dots * sizeof(float4));

		cudaMemcpy(d_color_data, h_color_data, dots * sizeof(uint32_t), cudaMemcpyHostToDevice);

		int height = sensor_calibration.color_camera_calibration.resolution_height;

		transform_to_color_cloud<<<height, 768>>>(d_point_cloud, d_color_data, d_color_point_cloud, d_color_points, d_dimensions);

		k4a_image_release(color_image);
	}

	cudaMemset(d_point_count, 0, sizeof(unsigned int));

	if (color_enabled)
		trim_and_conform_color<<<dots / 1024, 1024>>>(d_color_point_cloud, d_trimmed_point_cloud, d_color_points, d_trimmed_color_points, d_point_count);
	else
		trim_and_conform<<<dots / 1024, 1024>>>(d_point_cloud, d_trimmed_point_cloud, d_point_count);

	cudaMemcpy(h_point_count, d_point_count, sizeof(unsigned int), cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();

	k4a_image_release(depth_image);
}

uint32_t K4A_CudaPointCloud::GetPointCount()
{
	return (*h_point_count);
}

void K4A_CudaPointCloud::ResetPointCloud()
{
	if (color_enabled)
	{
		delete h_dimensions;
		cudaFree(d_color_data);
		cudaFree(d_color_points);
		cudaFree(d_color_point_cloud);
		cudaFree(d_trimmed_color_points);
		cudaFree(d_trimmed_point_cloud);
		cudaFree(d_point_count);
		cudaFree(d_dimensions);
	}
	cudaFree(d_depth_data);
	cudaFree(d_xy_table);
	cudaFree(d_point_cloud);
}

int K4A_CudaPointCloud::GetSkeletonCount()
{
	return skeleton_count;
}


void K4A_CudaPointCloud::SetSkeletonGroup(k4a_skeleton_group_t* group_ref)
{
	h_skeleton_group = group_ref;

	cudaHostRegister(h_skeleton_group, sizeof(k4a_skeleton_group_t), cudaHostRegisterMapped);
	cudaHostAlloc<k4a_skeleton_group_t>(&h_skeleton_group_unadjusted, sizeof(k4a_skeleton_group_t), cudaHostAllocWriteCombined);

	cudaHostGetDevicePointer<k4a_skeleton_group_t>(&d_skeleton_group, h_skeleton_group, 0);
	cudaHostGetDevicePointer<k4a_skeleton_group_t>(&d_skeleton_group_unadjusted, h_skeleton_group_unadjusted, 0);
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
		//printf("%zu bodies are detected!\n", num_bodies);

		for (int i = 0; i < num_bodies && i < MAX_TRACKED_SKELETONS; i++)
		{
			k4a_result_t get_body_result = k4abt_frame_get_body_skeleton(body_frame, i, &h_skeleton_group_unadjusted->skeletons[skeleton_count]);
			if (get_body_result == K4A_RESULT_SUCCEEDED)
			{
				skeleton_count++;
			}
		}

		for (int skel_id = skeleton_count; skel_id < MAX_TRACKED_SKELETONS; skel_id++) {
			h_skeleton_group_unadjusted->skeletons[skel_id] = k4abt_skeleton_t();
		}

		k4a_skeleton_adjust<<<4, 7>>>(skeleton_count, d_skeleton_group_unadjusted, d_skeleton_group);
		cudaDeviceSynchronize();

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