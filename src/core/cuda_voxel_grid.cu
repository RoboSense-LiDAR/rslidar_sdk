
#include "cuda_voxel_grid.cuh"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/execution_policy.h>

// Struct to hold point and its voxel ID
struct VoxelPoint
{
    long long voxel_id;
    CudaPointXYZI point;

    __host__ __device__
    bool operator<(const VoxelPoint& other) const
    {
        return voxel_id < other.voxel_id;
    }
};

// Kernel to compute voxel ID for each point
__global__ void computeVoxelIDsKernel(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    float inv_leaf_size,
    VoxelPoint* d_voxel_points)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    CudaPointXYZI p = d_input_cloud[idx];

    long long vx = static_cast<long long>(floor(p.x * inv_leaf_size));
    long long vy = static_cast<long long>(floor(p.y * inv_leaf_size));
    long long vz = static_cast<long long>(floor(p.z * inv_leaf_size));

    // Combine 3D voxel coordinates into a single 64-bit ID
    // This assumes coordinates fit within 21 bits each (approx +/- 1 million units with 0.01 leaf size)
    // Adjust bit shifts if larger coordinates or smaller leaf sizes are expected
    d_voxel_points[idx].voxel_id = (vx & 0x1FFFFF) | ((vy & 0x1FFFFF) << 21) | ((vz & 0x1FFFFF) << 42);
    d_voxel_points[idx].point = p;
}

// Host-side function to launch the CUDA kernel for voxel grid downsampling
cudaError_t voxelGridDownsampleGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    float leaf_size,
    CudaPointXYZI** d_output_cloud,
    size_t* num_output_points)
{
    if (num_input_points == 0 || leaf_size <= 0.0f)
    {
        *d_output_cloud = nullptr;
        *num_output_points = 0;
        return cudaSuccess;
    }

    cudaError_t err;

    int min_grid_size;
    int block_size;
    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, computeVoxelIDsKernel, 0, 0);
    const int BLOCK_SIZE = block_size;

    int num_blocks = (num_input_points + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // 1. Compute Voxel IDs
    thrust::device_vector<VoxelPoint> d_voxel_points(num_input_points);
    computeVoxelIDsKernel<<<num_blocks, BLOCK_SIZE>>>(
        d_input_cloud, num_input_points, 1.0f / leaf_size, thrust::raw_pointer_cast(d_voxel_points.data()));
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;
    err = cudaDeviceSynchronize();
    
    // 2. Sort by Voxel ID
    thrust::sort(thrust::device, d_voxel_points.begin(), d_voxel_points.end());

    // 3. Unique by Voxel ID (select first point in each voxel)
    auto new_end = thrust::unique(
        thrust::device, d_voxel_points.begin(), d_voxel_points.end(),
        [](const VoxelPoint& a, const VoxelPoint& b) __device__ { return a.voxel_id == b.voxel_id; });

    size_t unique_voxels = thrust::distance(d_voxel_points.begin(), new_end);
    *num_output_points = unique_voxels;

    if (unique_voxels > 0)
    {
        // 4. Copy unique points to output
        err = cudaMalloc((void**)d_output_cloud, unique_voxels * sizeof(CudaPointXYZI));
        if (err != cudaSuccess) return err;

        thrust::transform(thrust::device, d_voxel_points.begin(), d_voxel_points.begin() + unique_voxels,
                          thrust::raw_pointer_cast(*d_output_cloud),
                          [](const VoxelPoint& vp) __device__ { return vp.point; });
    }
    else
    {
        *d_output_cloud = nullptr;
    }

    return cudaSuccess;
}
