// dllmain.cpp : Defines the entry point for the DLL application.

#include "k4a_cuda.cuh"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>
#include <cinttypes>

K4A_CudaPointCloud* area;

extern "C" __declspec(dllexport) void init_cpc()
{
	area = new K4A_CudaPointCloud();
}

extern "C" __declspec(dllexport) void get_capture()
{
	area->GetCapture();
}

extern "C" __declspec(dllexport) float4* get_point_cloud()
{
	return area->GeneratePointCloud();
}

extern "C" __declspec(dllexport) int get_skeleton_count()
{
	return area->GetSkeletonCount();
}

extern "C" __declspec(dllexport) void get_skeletons()
{
	area->GetSkeletons();
}

extern "C" __declspec(dllexport) k4abt_skeleton_t get_skeleton(int skel_id)
{
	return area->GetSkeleton(skel_id);
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
