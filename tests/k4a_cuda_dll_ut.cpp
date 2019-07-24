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

typedef struct _k4a_skeleton_group_t
{
	k4abt_skeleton_t skeletons[MAX_TRACKED_SKELETONS];
} k4a_skeleton_group_t;

extern "C" __declspec(dllimport) void init_cpc();

extern "C" __declspec(dllimport) void get_capture();

extern "C" __declspec(dllimport) float4* get_point_cloud();

extern "C" __declspec(dllimport) int get_skeleton_count();

extern "C" __declspec(dllimport) void get_skeletons();

extern "C" __declspec(dllimport) void set_skeleton_group(k4a_skeleton_group_t* skeleton_group);

TEST(k4a_demo_win_ut, get_floats)
{
	init_cpc();

	get_capture();

	float4* floats = get_point_cloud();

	int nonzerocount = 0;
	int nancount = 0;

	for (int i = 0; i < 262144; i++) {
		if (!isnan<float>(floats[i].x) && floats[i].z != 0.0)
		{
			ASSERT_GT(floats[i].z, 0.0);
			ASSERT_LT(floats[i].z, 10.0);
			nonzerocount++;
		}
		else
		{
			nancount++;
		}
	}

	printf("%d %d", nonzerocount, nancount);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}