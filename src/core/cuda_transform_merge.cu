#include "cuda_transform_merge.cuh"
#include <cuda_runtime.h>

/**
 * @brief A simple CUDA kernel that transforms points from a single LiDAR and
 *        places them at a specific offset in the output buffer.
 */
__global__ void transformKernel(
    CudaPointXYZI* d_input_cloud,
    size_t num_points,
    CudaMatrix4f d_transform,
    CudaPointXYZI* d_output_cloud,
    size_t output_offset)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Boundary check to prevent out-of-bounds access
    if (idx >= num_points)
    {
        return;
    }

    // Read, transform, and write
    CudaPointXYZI p = d_input_cloud[idx];
    CudaPointXYZI transformed_p = d_transform.transform(p);
    d_output_cloud[output_offset + idx] = transformed_p;
}

/**
 * @brief Host-side wrapper to launch the simple transform kernel.
 */
cudaError_t transformAndCopyToOffsetGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_points,
    const CudaMatrix4f& h_transform,
    CudaPointXYZI* d_output_cloud,
    size_t output_offset)
{
    if (num_points == 0)
    {
        return cudaSuccess;
    }

    const int block_size = 256;
    int num_blocks = (num_points + block_size - 1) / block_size;

    transformKernel<<<num_blocks, block_size>>>(
        d_input_cloud, num_points, h_transform, d_output_cloud, output_offset);

    // Use cudaPeekAtLastError to check for launch errors without synchronizing.
    // This is faster than cudaGetLastError() as it doesn't require a sync.
    return cudaPeekAtLastError();
}
