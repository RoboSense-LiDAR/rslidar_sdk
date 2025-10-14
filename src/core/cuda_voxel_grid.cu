#include "cuda_voxel_grid.cuh"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/execution_policy.h>

// Helper struct to accumulate point sums and count
struct PointSum
{
    float x, y, z, max_intensity;
    int count;
};

// Functor to add two PointSum objects, used by reduce_by_key
struct PointAdd
{
    __host__ __device__
    PointSum operator()(const PointSum& a, const PointSum& b) const
    {
        PointSum result;
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
        result.count = a.count + b.count;
        result.max_intensity = fmaxf(a.max_intensity, b.max_intensity);
        return result;
    }
};

// Kernel to compute voxel ID and convert CudaPointXYZI to PointSum
__global__ void convertToPointSumKernel(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    float inv_leaf_size,
    long long* d_keys,
    PointSum* d_values)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    CudaPointXYZI p = d_input_cloud[idx];

    long long vx = static_cast<long long>(floorf(p.x * inv_leaf_size));
    long long vy = static_cast<long long>(floorf(p.y * inv_leaf_size));
    long long vz = static_cast<long long>(floorf(p.z * inv_leaf_size));

    d_keys[idx] = (vx & 0x1FFFFF) | ((vy & 0x1FFFFF) << 21) | ((vz & 0x1FFFFF) << 42);
    d_values[idx] = {p.x, p.y, p.z, p.intensity, 1};
}

// Kernel to compute the final average from the summed values
__global__ void computeAveragesKernel(
    PointSum* d_summed_values,
    size_t num_unique_voxels,
    CudaPointXYZI* d_output_cloud)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_unique_voxels) return;

    PointSum s = d_summed_values[idx];
    if (s.count > 0)
    {
        d_output_cloud[idx] = {s.x / s.count, s.y / s.count, s.z / s.count, s.max_intensity};
    }
}

// Host-side function for voxel grid downsampling using reduce_by_key
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
    const int block_size = 256;

    // 1. Allocate temporary storage for keys (voxel IDs) and values (PointSum)
    thrust::device_vector<long long> d_keys(num_input_points);
    thrust::device_vector<PointSum> d_values(num_input_points);

    // 2. Launch kernel to compute voxel IDs and convert points to PointSum objects
    int num_blocks_1 = (num_input_points + block_size - 1) / block_size;
    convertToPointSumKernel<<<num_blocks_1, block_size>>>(
        d_input_cloud, num_input_points, 1.0f / leaf_size,
        thrust::raw_pointer_cast(d_keys.data()),
        thrust::raw_pointer_cast(d_values.data()));
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 3. Sort values by keys to group points from the same voxel together
    thrust::sort_by_key(thrust::device, d_keys.begin(), d_keys.end(), d_values.begin());
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 4. Allocate output storage for unique keys and summed values
    thrust::device_vector<long long> d_unique_keys(num_input_points);
    thrust::device_vector<PointSum> d_summed_values(num_input_points);

    // 5. Reduce by key to sum points within each voxel
    auto end_pair = thrust::reduce_by_key(
        thrust::device,
        d_keys.begin(), d_keys.end(),
        d_values.begin(),
        d_unique_keys.begin(),
        d_summed_values.begin(),
        thrust::equal_to<long long>(),
        PointAdd());
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    *num_output_points = end_pair.first - d_unique_keys.begin();

    if (*num_output_points > 0)
    {
        // 6. Allocate final output cloud
        err = cudaMalloc((void**)d_output_cloud, *num_output_points * sizeof(CudaPointXYZI));
        if (err != cudaSuccess) return err;

        // 7. Launch kernel to compute the final centroids from the sums
        int num_blocks_2 = (*num_output_points + block_size - 1) / block_size;
        computeAveragesKernel<<<num_blocks_2, block_size>>>(
            thrust::raw_pointer_cast(d_summed_values.data()),
            *num_output_points,
            *d_output_cloud);
        err = cudaGetLastError();
        if (err != cudaSuccess) { cudaFree(*d_output_cloud); return err; }
    }
    else
    {
        *d_output_cloud = nullptr;
    }

    return cudaDeviceSynchronize();
}
