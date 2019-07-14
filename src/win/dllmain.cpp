// dllmain.cpp : Defines the entry point for the DLL application.

#include "k4a_cuda.cuh"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>
#include <cinttypes>

K4A_CudaPointCloud* K4A_CPC;

extern "C" __declspec(dllexport) void init_cpc()
{
	K4A_CPC = new K4A_CudaPointCloud();
}

extern "C" __declspec(dllexport) void get_capture()
{
	K4A_CPC->GetCapture();
}

extern "C" __declspec(dllexport) float4* get_point_cloud()
{
	return K4A_CPC->GeneratePointCloud();
}

extern "C" __declspec(dllexport) float3* get_skeleton_joints()
{
	return K4A_CPC->GetSkeletonJoints();
}

extern "C" __declspec(dllexport) int get_skeleton_count()
{
	return K4A_CPC->GetSkeletonCount();
}

extern "C" __declspec(dllexport) float4* get_skeleton_joints_rots()
{
	return K4A_CPC->GetSkeletonJointsRots();
}

BOOL APIENTRY DllMain(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		break;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
