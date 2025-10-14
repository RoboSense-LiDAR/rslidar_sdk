#ifndef CUDA_POINT_TYPES_CUH
#define CUDA_POINT_TYPES_CUH

#include <cstddef>

#ifndef __CUDACC__
  #ifndef __host__
    #define __host__
  #endif
  #ifndef __device__
    #define __device__
  #endif
#endif

#pragma pack(push, 1)
// Point structure for GPU memory
struct CudaPointXYZI
{
    float x, y, z;
    uint8_t intensity;
};
#pragma pack(pop)

// Simple 4x4 matrix structure for CUDA kernels
struct CudaMatrix4f
{
    float data[16]; // Column-major order, like Eigen

    // Host-side conversion from Eigen::Matrix4f
    __host__ void fromEigen(const float* eigen_data)
    {
        for (int i = 0; i < 16; ++i)
        {
            data[i] = eigen_data[i];
        }
    }

    // Device-side multiplication with a point
    __device__ CudaPointXYZI transform(const CudaPointXYZI& p) const
    {
        CudaPointXYZI res;
        res.x = data[0] * p.x + data[4] * p.y + data[8] * p.z + data[12];
        res.y = data[1] * p.x + data[5] * p.y + data[9] * p.z + data[13];
        res.z = data[2] * p.x + data[6] * p.y + data[10] * p.z + data[14];
        res.intensity = p.intensity; // Intensity usually doesn't change with geometric transform
        return res;
    }
};

// LaserScan parameters for CUDA kernels
struct CudaLaserScanParams
{
    float angle_min, angle_max, angle_increment;
    float range_min, range_max;
    float min_height, max_height;
    size_t num_beams;
};

#endif // CUDA_POINT_TYPES_CUH