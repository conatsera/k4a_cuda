#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "k4a_cuda.cuh"

#define MEMORY_ALIGNMENT  4096
#define ALIGN_UP(x,size) ( ((size_t)x+(size-1))&(~(size-1)) )

using namespace testing;

static K4A_CudaPointCloud* g_CPC;

class k4a_demo_ut : public ::testing::Test
{
protected:
	float4* floats;
	uint32_t* colors;

	void SetUp() override
	{
		if (g_CPC == nullptr)
			g_CPC = new K4A_CudaPointCloud(true, false);
	}

	void TearDown() override
	{

	}
};

TEST_F(k4a_demo_ut, get_skeletons)
{
	if (false)
	{
		for (int i = 0; i < 256; i++)
		{
			g_CPC->GetCapture();
			k4a_skeleton_group_t* skeleton_group = new k4a_skeleton_group_t();
			g_CPC->SetSkeletonGroup(skeleton_group);
			g_CPC->GetSkeletons();
			int skel_count = g_CPC->GetSkeletonCount();
			if (skel_count != 0) {
				for (int skel_id = 0; skel_id < skel_count; skel_id++) {
					printf("%f\n", skeleton_group->skeletons[skel_id].joints[K4ABT_JOINT_WRIST_RIGHT].position.xyz.x);
				}
			}
		}
	}
}


TEST_F(k4a_demo_ut, get_floats)
{
	int dots = g_CPC->GetMaxPointCount();
	floats = (float4*)malloc(sizeof(float4) * dots);
	colors = (uint32_t*)malloc(sizeof(uint32_t) * dots);
	float4* aligned_floats = (float4*)ALIGN_UP(floats, MEMORY_ALIGNMENT);
	uint32_t* aligned_colors = (uint32_t*)ALIGN_UP(colors, MEMORY_ALIGNMENT);
	g_CPC->SetupPointCloud(aligned_floats, aligned_colors);
	for (int i = 0; i < 1024; i++)
	{
		g_CPC->GetCapture();
		g_CPC->GeneratePointCloud();
		//g_CPC->GetPointColors();
		uint32_t count = g_CPC->GetPointCount();

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