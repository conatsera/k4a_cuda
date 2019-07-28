// dllmain.cpp : Defines the entry point for the DLL application.

#include "k4a_cuda.cuh"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>
#include <cinttypes>

K4A_CudaPointCloud* area;

extern "C" __declspec(dllexport) void init_cpc(bool color, bool body_tracking)
{
	area = new K4A_CudaPointCloud(color, body_tracking);
}

extern "C" __declspec(dllexport) void get_capture()
{
	area->GetCapture();
}

extern "C" __declspec(dllexport) void setup_point_cloud()
{
	area->SetupPointCloud();
}

extern "C" __declspec(dllexport) float4* get_point_cloud()
{
	return area->GeneratePointCloud();
}

extern "C" __declspec(dllexport) uint32_t* get_point_colors()
{
	return area->GetPointColors();
}

extern "C" __declspec(dllexport) int get_skeleton_count()
{
	return area->GetSkeletonCount();
}

extern "C" __declspec(dllexport) void get_skeletons()
{
	area->GetSkeletons();
}

extern "C" __declspec(dllexport) void set_skeleton_group(k4a_skeleton_group_t* skeleton_group)
{
	area->SetSkeletonGroup(skeleton_group);
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
