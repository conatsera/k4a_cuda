# k4a_cuda
A K4A library that wraps functionality from the K4A and K4ABT C SDKs for use in Unity C# scripts.
This library performs the point cloud calculation in CUDA as described in the SDK examples.

# Script example
```c#
    using System;
using System.Runtime.InteropServices;
using UnityEngine;

unsafe public class GenerateCloud : MonoBehaviour
{
    ParticleSystem ps;


    [StructLayout(LayoutKind.Sequential)]
    public struct float4
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }

    [DllImport("k4a_cuda_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern void init_cpc(bool color, bool body_tracking);

    [DllImport("k4a_cuda_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern void get_capture();

    [DllImport("k4a_cuda_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern void setup_point_cloud();

    [DllImport("k4a_cuda_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern float4* get_point_cloud();

    [DllImport("k4a_cuda_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern uint* get_point_colors();

    void Start()
    {
        ps = GetComponent<ParticleSystem>();

        // For color change to init_cpc(true, false) and uncomment color data lines below
        init_cpc(false, false);

        setup_point_cloud();
    }

    void Update()
    {
        get_capture();
        float4* points = get_point_cloud();
        //uint* point_colors = get_point_colors();
        // Note: A particle system may work in a pinch, but it is not designed to handle this amount of particles
        // For a colored point cloud, you will have to cut 1/4 to 1/2 of the particles or Unity will not load
        for (int i = 0; i < 262144; i++)
        {
            if (!float.IsNaN(points[i].x) && points[i].x != 0)
            {
                //byte[] color_bits = BitConverter.GetBytes(point_colors[i]);
                
                ParticleSystem.EmitParams ep = new ParticleSystem.EmitParams
                {
                    position = new Vector3(points[i].x, points[i].y, points[i].z),
                    //startColor = new Color32(color_bits[2], color_bits[1], color_bits[0], color_bits[3])
                };

                ps.Emit(ep, 1);
            }
        }
    }
}
```

## Notes
* This project has been build and tested using CUDA 10.1 and Visual Studio 16 2019
* dnn_model.onnx is not path-searched and is required with your application (in "\<output\>/\<Name\>_Data/" for Unity)
* Calculating and displaying approximately 6M points per second, running a DNN model, and running a game engine does require a fairly powerful system. I've had success (albeit with a 1/4 to 1/2 second delay) on a GTX 960M and with minimal delay on GTX 1080
* Set your CUDA version with -DCMAKE_CUDA_COMPILER="\<path-to-cuda/bin\>/nvcc.exe""

## Further work
* As with the last point above, there are several place I could optimize my code to improve the overall performance. This is my first C++ project on a system with more than 1MB of RAM, so any suggestions/pull requests are welcome.
* Use a remote system for processing the point cloud and body tracking
