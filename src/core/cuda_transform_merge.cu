#include "cuda_transform_merge.cuh"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/binary_search.h>
#include <thrust/execution_policy.h>

// CUDA kernel to transform and merge points
__global__ void transformAndMergeKernel(
    CudaPointXYZI** d_input_clouds,
    size_t* d_input_counts,
    size_t* d_prefix_sums, // New: prefix sums of point counts
    CudaMatrix4f* d_transforms,
    size_t num_lidars,
    CudaPointXYZI* d_output_cloud)
{
    size_t global_idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Find which lidar this global_idx belongs to using a manual binary search (like thrust::upper_bound)
    // This correctly interprets d_prefix_sums as starting offsets.
    size_t lidar_idx = 0;
    size_t low = 0, high = num_lidars;
    while (low < high)
    {
        size_t mid = low + (high - low) / 2;
        if (global_idx >= d_prefix_sums[mid])
        {
            lidar_idx = mid;
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }

    size_t local_idx = global_idx - (lidar_idx == 0 ? 0 : d_prefix_sums[lidar_idx - 1]);

    CudaPointXYZI p = d_input_clouds[lidar_idx][local_idx];
    CudaMatrix4f transform = d_transforms[lidar_idx];

    // Apply transform
    CudaPointXYZI transformed_p = transform.transform(p);

    // Write to output
    d_output_cloud[global_idx] = transformed_p;
}

// Host-side function to launch the CUDA kernel
cudaError_t transformAndMergeGPU(
    const std::vector<CudaPointXYZI*>& h_input_clouds,
    const std::vector<size_t>& h_input_counts,
    const std::vector<size_t>& h_prefix_sums, // New: prefix sums of point counts
    const std::vector<CudaMatrix4f>& h_transforms,
    CudaPointXYZI* d_output_cloud,
    size_t total_output_points)
{
    cudaError_t err;

    // Copy host vectors to device
    CudaPointXYZI** d_input_clouds;
    err = cudaMalloc((void**)&d_input_clouds, h_input_clouds.size() * sizeof(CudaPointXYZI*));
    if (err != cudaSuccess) return err;
    err = cudaMemcpy(d_input_clouds, h_input_clouds.data(), h_input_clouds.size() * sizeof(CudaPointXYZI*), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); return err; }

    size_t* d_input_counts_ptr;
    err = cudaMalloc((void**)&d_input_counts_ptr, h_input_counts.size() * sizeof(size_t));
    if (err != cudaSuccess) { cudaFree(d_input_clouds); return err; }
    err = cudaMemcpy(d_input_counts_ptr, h_input_counts.data(), h_input_counts.size() * sizeof(size_t), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_input_counts_ptr); return err; }

    size_t* d_prefix_sums_ptr; // New: device pointer for prefix sums
    err = cudaMalloc((void**)&d_prefix_sums_ptr, h_prefix_sums.size() * sizeof(size_t));
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_input_counts_ptr); return err; }
    err = cudaMemcpy(d_prefix_sums_ptr, h_prefix_sums.data(), h_prefix_sums.size() * sizeof(size_t), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_input_counts_ptr); cudaFree(d_prefix_sums_ptr); return err; }

    CudaMatrix4f* d_transforms_ptr;
    err = cudaMalloc((void**)&d_transforms_ptr, h_transforms.size() * sizeof(CudaMatrix4f));
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_input_counts_ptr); cudaFree(d_prefix_sums_ptr); return err; }
    err = cudaMemcpy(d_transforms_ptr, h_transforms.data(), h_transforms.size() * sizeof(CudaMatrix4f), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_input_counts_ptr); cudaFree(d_prefix_sums_ptr); cudaFree(d_transforms_ptr); return err; }

    // Launch kernel
    int min_grid_size;
    int block_size = 256;
    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, transformAndMergeKernel, 0, 0);
    const int BLOCK_SIZE = block_size;
    int num_blocks = (total_output_points + BLOCK_SIZE - 1) / BLOCK_SIZE;
    transformAndMergeKernel<<<num_blocks, BLOCK_SIZE>>>(
        d_input_clouds, d_input_counts_ptr, d_prefix_sums_ptr, d_transforms_ptr, h_input_clouds.size(), d_output_cloud);

    // Check for CUDA errors
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "CUDA kernel launch failed: %s\n", cudaGetErrorString(err));
    }
    err = cudaDeviceSynchronize();
    
    // Free device memory for temporary arrays
    cudaFree(d_input_clouds);
    cudaFree(d_input_counts_ptr);
    cudaFree(d_prefix_sums_ptr);
    cudaFree(d_transforms_ptr);

    return err;
}