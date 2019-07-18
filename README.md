# k4a_cuda
A K4A library that wraps functionality from the K4A and K4ABT C SDKs for use in Unity C# scripts.
This library performs the point cloud calculation in CUDA as described in the SDK examples.

# Script example
```c#
    [DllImport("k4a_demo_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern void init_cpc();

    [DllImport("k4a_demo_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern void get_capture();

    [DllImport("k4a_demo_win.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    static extern float4* get_point_cloud();
    
    void Start()
    {
        init_cpc();
    }
    
    void Update()
    {
      get_capture();
      float4* points = get_point_cloud();
      
      for (int i = 0; i < 262144; i++)
      {
        if (!float.IsNaN(points[i].x) && points[i].x != 0)
        {
          ParticleSystem.EmitParams ep = new ParticleSystem.EmitParams
          {
            position = new Vector3(points[i].x, -points[i].y, points[i].z),
            startSize = 0.01F,
            startLifetime = 0.2F,
            startColor = Color.green
          };

          ps.Emit(ep, 1);
        }
      }
    }
```

## Notes
* This project has been build and tested using CUDA 10.1 and Visual Studio 16 2019.
* dnn_model.onnx is not path-searched and is required with your application (in "\<output\>/\<Name\>_Data/" for Unity)
