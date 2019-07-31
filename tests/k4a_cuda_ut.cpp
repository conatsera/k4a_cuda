#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "k4a_cuda.cuh"

using namespace testing;

static K4A_CudaPointCloud* g_CPC;

class k4a_demo_ut : public ::testing::Test
{
protected:
	float4* floats;

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
	g_CPC->SetupPointCloud();
	for (int i = 0; i < 1024; i++)
	{
		g_CPC->GetCapture();
		floats = g_CPC->GeneratePointCloud();
		uint32_t* colors = g_CPC->GetPointColors();
		uint32_t count = g_CPC->GetPointCount();

		//printf("%d\n", count);
	}
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}