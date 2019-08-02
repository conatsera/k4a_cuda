#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "k4abttypes.h"

struct float4 {
	float x;
	float y;
	float z;
	float w;
};

#define MAX_TRACKED_SKELETONS 4

#define MEMORY_ALIGNMENT  4096
#define ALIGN_UP(x,size) ( ((size_t)x+(size-1))&(~(size-1)) )

typedef struct _k4a_skeleton_group_t
{
	k4abt_skeleton_t skeletons[MAX_TRACKED_SKELETONS];
} k4a_skeleton_group_t;

extern "C" __declspec(dllimport) void init_cpc(bool color, bool body_tracking);

extern "C" __declspec(dllimport) void get_capture();

extern "C" __declspec(dllimport) void setup_point_cloud(float4** point_cloud, uint32_t** point_colors);

extern "C" __declspec(dllimport) void get_point_cloud();

extern "C" __declspec(dllimport) uint32_t get_point_count();

extern "C" __declspec(dllimport) int get_max_point_count();

extern "C" __declspec(dllimport) int get_skeleton_count();

extern "C" __declspec(dllimport) void get_skeletons();

extern "C" __declspec(dllimport) void set_skeleton_group(k4a_skeleton_group_t* skeleton_group);

TEST(k4a_demo_win_ut, get_floats)
{
	init_cpc(true, false);

	int dots = get_max_point_count();
	float4* floats = (float4*)malloc(sizeof(float4) * dots);
	uint32_t* colors = (uint32_t*)malloc(sizeof(uint32_t) * dots);
	float4* aligned_floats = (float4*)ALIGN_UP(floats, MEMORY_ALIGNMENT);
	uint32_t* aligned_colors = (uint32_t*)ALIGN_UP(colors, MEMORY_ALIGNMENT);
	setup_point_cloud(&aligned_floats, &aligned_colors);
	for (int i = 0; i < 1024; i++)
	{
		get_capture();
		get_point_cloud();
		//g_CPC->GetPointColors();
		uint32_t count = get_point_count();

		int check = 0;
		for (int i = 0; i < count; i++)
		{
			if (aligned_floats[i].x != nanf("") && aligned_floats[i].z != 0.0F && aligned_colors[i] != 0)
				check++;
		}

		ASSERT_EQ(count, check);
	}
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}