#ifndef CUDA_TRANSFORM_MERGE_CUH
#define CUDA_TRANSFORM_MERGE_CUH

#include "cuda_point_types.cuh"
#include <vector>

/**
 * @brief Transforms a single point cloud on the GPU and copies the result to a specific offset
 *        in a larger output buffer. This is a simpler, more robust alternative to a single complex kernel.
 * @param d_input_cloud Pointer to the input point cloud data on the GPU.
 * @param num_points The number of points in the input cloud.
 * @param h_transform The transformation matrix (on the host) to apply.
 * @param d_output_cloud The larger destination buffer on the GPU.
 * @param output_offset The starting index in the output buffer where results should be written.
 * @return cudaError_t error code.
 */
cudaError_t transformAndCopyToOffsetGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_points,
    const CudaMatrix4f& h_transform,
    CudaPointXYZI* d_output_cloud,
    size_t output_offset);

#endif // CUDA_TRANSFORM_MERGE_CUH
