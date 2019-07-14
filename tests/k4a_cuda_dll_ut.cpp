#include <gtest/gtest.h>
#include <gmock/gmock.h>

struct float4 {
	float x;
	float y;
	float z;
	float w;
};

extern "C" __declspec(dllimport) float4* get_point_cloud();

extern "C" __declspec(dllimport) void init_cpc();

TEST(k4a_cuda_dll_ut, get_floats)
{
	init_cpc();

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