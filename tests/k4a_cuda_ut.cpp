#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "k4a_cuda.cuh"

using namespace testing;

static K4A_CudaPointCloud* g_K4A_CPC;

class k4a_cuda_ut : public ::testing::Test
{
protected:
	float4* floats;

	void SetUp() override
	{
		if (g_K4A_CPC == nullptr)
			g_K4A_CPC = new K4A_CudaPointCloud();
	}
	
	void TearDown() override
	{

	}
};

TEST_F(k4a_cuda_ut, get_skeletons)
{
	for (int i = 0; i < 1024; i++)
	{
		g_K4A_CPC->GetCapture();
		float3* joints = g_K4A_CPC->GetSkeletonJoints();
		int skeleton_count = g_K4A_CPC->GetSkeletonCount();
		//float4* joint_rots = g_K4A_CPC->GetSkeletonJointsRots();
		for (int j = 0; j < skeleton_count; j++)
		{
			printf("%f\n", joints[j * K4ABT_JOINT_WRIST_RIGHT].x);
		}
	}
}

TEST_F(k4a_cuda_ut, get_floats)
{
	g_K4A_CPC->GetCapture();
	floats = g_K4A_CPC->GeneratePointCloud();

	int nonzerocount = 0;
	int nancount = 0;

	for (int i = 0; i < 262144; i++) {
		if (!isnan<float>(floats[i].x) && floats[i].z != 0.0)
		{
			ASSERT_GT(floats[i].z, 0.0);
			ASSERT_LT(floats[i].z, 15.0);
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