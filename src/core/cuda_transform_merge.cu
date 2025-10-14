#include "cuda_transform_merge.cuh"
#include <cuda_runtime.h>

// CUDA kernel to transform and merge points
__global__ void transformAndMergeKernel(
    CudaPointXYZI** d_input_clouds,
    size_t* d_prefix_sums,
    CudaMatrix4f* d_transforms,
    size_t num_lidars,
    CudaPointXYZI* d_output_cloud,
    size_t total_points) // Argument for boundary check
{
    size_t global_idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Boundary check to prevent out-of-bounds access
    if (global_idx >= total_points)
    {
        return;
    }

    // Find which lidar this global_idx belongs to using a manual binary search
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

    size_t local_idx = global_idx - d_prefix_sums[lidar_idx];

    CudaPointXYZI p = d_input_clouds[lidar_idx][local_idx];
    CudaMatrix4f transform = d_transforms[lidar_idx];

    CudaPointXYZI transformed_p = transform.transform(p);

    d_output_cloud[global_idx] = transformed_p;
}

// Host-side function to launch the CUDA kernel
cudaError_t transformAndMergeGPU(
    const std::vector<CudaPointXYZI*>& h_input_clouds,
    const std::vector<size_t>& h_input_counts,
    const std::vector<size_t>& h_prefix_sums,
    const std::vector<CudaMatrix4f>& h_transforms,
    CudaPointXYZI* d_output_cloud,
    size_t total_output_points)
{
    cudaError_t err;

    CudaPointXYZI** d_input_clouds;
    err = cudaMalloc((void**)&d_input_clouds, h_input_clouds.size() * sizeof(CudaPointXYZI*));
    if (err != cudaSuccess) return err;
    err = cudaMemcpy(d_input_clouds, h_input_clouds.data(), h_input_clouds.size() * sizeof(CudaPointXYZI*), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); return err; }

    size_t* d_prefix_sums_ptr;
    err = cudaMalloc((void**)&d_prefix_sums_ptr, h_prefix_sums.size() * sizeof(size_t));
    if (err != cudaSuccess) { cudaFree(d_input_clouds); return err; }
    err = cudaMemcpy(d_prefix_sums_ptr, h_prefix_sums.data(), h_prefix_sums.size() * sizeof(size_t), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_prefix_sums_ptr); return err; }

    CudaMatrix4f* d_transforms_ptr;
    err = cudaMalloc((void**)&d_transforms_ptr, h_transforms.size() * sizeof(CudaMatrix4f));
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_prefix_sums_ptr); return err; }
    err = cudaMemcpy(d_transforms_ptr, h_transforms.data(), h_transforms.size() * sizeof(CudaMatrix4f), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_input_clouds); cudaFree(d_prefix_sums_ptr); cudaFree(d_transforms_ptr); return err; }

    const int block_size = 256;
    int num_blocks = (total_output_points + block_size - 1) / block_size;
    
    transformAndMergeKernel<<<num_blocks, block_size>>>(
        d_input_clouds, d_prefix_sums_ptr, d_transforms_ptr, h_input_clouds.size(), d_output_cloud, total_output_points);

    err = cudaGetLastError();
    if (err == cudaSuccess)
    {
        err = cudaDeviceSynchronize();
    }
    
    cudaFree(d_input_clouds);
    cudaFree(d_prefix_sums_ptr);
    cudaFree(d_transforms_ptr);

    return err;
}
