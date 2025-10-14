#include "cuda_voxel_grid.cuh"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/execution_policy.h>

// Struct to hold point and its voxel ID for sorting
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

    long long vx = static_cast<long long>(floorf(p.x * inv_leaf_size));
    long long vy = static_cast<long long>(floorf(p.y * inv_leaf_size));
    long long vz = static_cast<long long>(floorf(p.z * inv_leaf_size));

    // A simple and robust way to create a unique 64-bit ID from 3 21-bit coordinates
    d_voxel_points[idx].voxel_id = (vx & 0x1FFFFF) | ((vy & 0x1FFFFF) << 21) | ((vz & 0x1FFFFF) << 42);
    d_voxel_points[idx].point = p;
}

// Kernel to find the start index of each unique voxel in the sorted list
__global__ void findVoxelStartsKernel(
    VoxelPoint* d_sorted_voxel_points,
    size_t num_input_points,
    int* d_voxel_start_flags)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    // The first point always starts a new voxel
    if (idx == 0)
    {
        d_voxel_start_flags[idx] = 1;
        return;
    }

    // A point starts a new voxel if its ID is different from the previous one
    d_voxel_start_flags[idx] = (d_sorted_voxel_points[idx].voxel_id != d_sorted_voxel_points[idx - 1].voxel_id);
}

// Kernel to compute the centroid for each voxel
__global__ void computeCentroidsKernel(
    VoxelPoint* d_sorted_voxel_points,
    int* d_voxel_start_indices,
    size_t num_unique_voxels,
    CudaPointXYZI* d_output_cloud)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_unique_voxels) return;

    int start_index = d_voxel_start_indices[idx];
    // The end index is the start of the next voxel, or the total number of points for the last voxel
    int end_index = (idx == num_unique_voxels - 1) ? d_voxel_start_indices[num_unique_voxels] : d_voxel_start_indices[idx + 1];

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f, sum_intensity = 0.0f;
    for (int i = start_index; i < end_index; ++i)
    {
        sum_x += d_sorted_voxel_points[i].point.x;
        sum_y += d_sorted_voxel_points[i].point.y;
        sum_z += d_sorted_voxel_points[i].point.z;
        sum_intensity += d_sorted_voxel_points[i].point.intensity;
    }

    int count = end_index - start_index;
    d_output_cloud[idx].x = sum_x / count;
    d_output_cloud[idx].y = sum_y / count;
    d_output_cloud[idx].z = sum_z / count;
    d_output_cloud[idx].intensity = sum_intensity / count;
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

    // 1. Create VoxelPoint structs (VoxelID + Point)
    thrust::device_vector<VoxelPoint> d_voxel_points(num_input_points);
    int num_blocks_1 = (num_input_points + block_size - 1) / block_size;
    computeVoxelIDsKernel<<<num_blocks_1, block_size>>>(
        d_input_cloud, num_input_points, 1.0f / leaf_size, thrust::raw_pointer_cast(d_voxel_points.data()));
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 2. Sort points by VoxelID to group them
    thrust::sort(thrust::device, d_voxel_points.begin(), d_voxel_points.end());
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 3. Find the starting index of each unique voxel
    thrust::device_vector<int> d_voxel_start_flags(num_input_points);
    int num_blocks_2 = (num_input_points + block_size - 1) / block_size;
    findVoxelStartsKernel<<<num_blocks_2, block_size>>>(
        thrust::raw_pointer_cast(d_voxel_points.data()), num_input_points, thrust::raw_pointer_cast(d_voxel_start_flags.data()));
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // 4. Perform an exclusive scan (prefix sum) on the flags to get the start indices
    thrust::device_vector<int> d_voxel_start_indices(num_input_points + 1);
    thrust::exclusive_scan(thrust::device, d_voxel_start_flags.begin(), d_voxel_start_flags.end(), d_voxel_start_indices.begin());
    err = cudaGetLastError();
    if (err != cudaSuccess) return err;

    // The total number of unique voxels is the sum of all flags
    int h_num_unique_voxels;
    cudaMemcpy(&h_num_unique_voxels, thrust::raw_pointer_cast(d_voxel_start_indices.data()) + num_input_points, sizeof(int), cudaMemcpyDeviceToHost);
    *num_output_points = h_num_unique_voxels;

    if (*num_output_points > 0)
    {
        // 5. Allocate output cloud
        err = cudaMalloc((void**)d_output_cloud, *num_output_points * sizeof(CudaPointXYZI));
        if (err != cudaSuccess) return err;

        // 6. Launch kernel to compute centroids
        int num_blocks_3 = (*num_output_points + block_size - 1) / block_size;
        computeCentroidsKernel<<<num_blocks_3, block_size>>>(
            thrust::raw_pointer_cast(d_voxel_points.data()),
            thrust::raw_pointer_cast(d_voxel_start_indices.data()),
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
