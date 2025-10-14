#include "cuda_voxel_grid.cuh"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

// Voxel ID (key)
using VoxelId = long long;

// Point Sum and Count (value for reduction)
struct PointSumAndCount
{
    float x, y, z, intensity;
    unsigned int count;

    __host__ __device__
    PointSumAndCount() : x(0), y(0), z(0), intensity(0), count(0) {}

    __host__ __device__
    PointSumAndCount(float px, float py, float pz, float pint, unsigned int c)
        : x(px), y(py), z(pz), intensity(pint), count(c) {}
};

// Binary operator for thrust::reduce_by_key
struct PointSumFunctor
{
    __host__ __device__
    PointSumAndCount operator()(const PointSumAndCount& a, const PointSumAndCount& b) const
    {
        return PointSumAndCount(a.x + b.x, a.y + b.y, a.z + b.z, a.intensity + b.intensity, a.count + b.count);
    }
};

// Kernel to compute voxel ID for each point
__global__ void computeVoxelDataKernel(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    float inv_leaf_size,
    VoxelId* d_voxel_ids,
    PointSumAndCount* d_point_sums)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    CudaPointXYZI p = d_input_cloud[idx];

    long long vx = static_cast<long long>(floorf(p.x * inv_leaf_size));
    long long vy = static_cast<long long>(floorf(p.y * inv_leaf_size));
    long long vz = static_cast<long long>(floorf(p.z * inv_leaf_size));

    d_voxel_ids[idx] = (vx & 0x1FFFFF) | ((vy & 0x1FFFFF) << 21) | ((vz & 0x1FFFFF) << 42);
    d_point_sums[idx] = PointSumAndCount(p.x, p.y, p.z, p.intensity, 1);
}

// Kernel to compute the final centroid from the sum and count
__global__ void computeCentroidKernel(
    PointSumAndCount* d_reduced_sums,
    size_t num_unique_voxels,
    CudaPointXYZI* d_output_cloud)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_unique_voxels) return;

    PointSumAndCount psc = d_reduced_sums[idx];
    if (psc.count > 0)
    {
        d_output_cloud[idx].x = psc.x / psc.count;
        d_output_cloud[idx].y = psc.y / psc.count;
        d_output_cloud[idx].z = psc.z / psc.count;
        d_output_cloud[idx].intensity = psc.intensity / psc.count;
    }
}

// Host-side function for voxel grid downsampling
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
    int num_blocks = (num_input_points + block_size - 1) / block_size;

    // 1. Allocate temporary storage
    thrust::device_vector<VoxelId> d_voxel_ids(num_input_points);
    thrust::device_vector<PointSumAndCount> d_point_sums(num_input_points);
    
    // 2. Compute Voxel ID and initial sum (count=1) for each point
    computeVoxelDataKernel<<<num_blocks, block_size>>>(
        d_input_cloud, num_input_points, 1.0f / leaf_size, 
        thrust::raw_pointer_cast(d_voxel_ids.data()), 
        thrust::raw_pointer_cast(d_point_sums.data()));
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 3. Sort by Voxel ID to group points in the same voxel together
    thrust::sort_by_key(thrust::device, d_voxel_ids.begin(), d_voxel_ids.end(), d_point_sums.begin());
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 4. Reduce by key to sum up points within each unique voxel.
    // This is an in-place reduction: the unique keys will be written back into d_voxel_ids.
    auto end_iterators = thrust::reduce_by_key(
        thrust::device,
        d_voxel_ids.begin(), d_voxel_ids.end(),
        d_point_sums.begin(),
        d_voxel_ids.begin(), // Output keys (in-place)
        d_reduced_sums.begin(),
        thrust::equal_to<VoxelId>(),
        PointSumFunctor());

    size_t unique_voxels = thrust::distance(d_voxel_ids.begin(), end_iterators.first);
    *num_output_points = unique_voxels;

    if (unique_voxels > 0)
    {
        // 5. Allocate final output buffer
        err = cudaMalloc((void**)d_output_cloud, unique_voxels * sizeof(CudaPointXYZI));
        if (err != cudaSuccess) return err;

        // 6. Compute the centroid for each voxel
        int num_centroid_blocks = (unique_voxels + block_size - 1) / block_size;
        computeCentroidKernel<<<num_centroid_blocks, block_size>>>(
            thrust::raw_pointer_cast(d_reduced_sums.data()), unique_voxels, *d_output_cloud);
        err = cudaGetLastError();
        if (err != cudaSuccess) { cudaFree(*d_output_cloud); return err; }
    }
    else
    {
        *d_output_cloud = nullptr;
    }

    return cudaDeviceSynchronize();
}