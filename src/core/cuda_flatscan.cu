
#include "cuda_flatscan.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

// Kernel to initialize ranges array with infinity
__global__ void initRangesKernel(float* d_ranges, size_t num_beams)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_beams)
    {
        d_ranges[idx] = INFINITY;
    }
}

// Kernel to generate LaserScan data from a point cloud
__global__ void generateFlatScanKernel(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    CudaLaserScanParams params,
    float* d_ranges)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    CudaPointXYZI p = d_input_cloud[idx];

    // Check height range
    if (p.z >= params.min_height && p.z <= params.max_height)
    {
        float angle = atan2f(p.y, p.x);
        float range = sqrtf(p.x * p.x + p.y * p.y);

        // Check range limits
        if (range >= params.range_min && range <= params.range_max)
        {
            // Calculate index for the angular bin
            int bin_idx = static_cast<int>((angle - params.angle_min) / params.angle_increment);

            if (bin_idx >= 0 && bin_idx < params.num_beams)
            {
                // Atomically update the minimum range for this bin using a CAS loop
                float* address = &d_ranges[bin_idx];
                float old_val = *address;
                while (range < old_val)
                {
                    unsigned int* address_as_uint = (unsigned int*)address;
                    unsigned int old_int = __float_as_uint(old_val);
                    unsigned int new_int = __float_as_uint(range);
                    unsigned int returned_int = atomicCAS(address_as_uint, old_int, new_int);
                    if (returned_int == old_int)
                    {
                        break; // Success
                    }
                    old_val = __uint_as_float(returned_int); // Update and retry
                }
            }
        }
    }
}

// Host-side function to launch the CUDA kernel for LaserScan generation
cudaError_t generateFlatScanGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    const CudaLaserScanParams& params,
    float** d_ranges_output)
{
    cudaError_t err;

    // Allocate GPU memory for ranges and initialize to infinity
    float* d_ranges;
    err = cudaMalloc((void**)&d_ranges, params.num_beams * sizeof(float));
    if (err != cudaSuccess) return err;

    int min_grid_size;
    int block_size;
    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, generateFlatScanKernel, 0, 0);
    const int BLOCK_SIZE = block_size;

    int num_blocks_init = (params.num_beams + BLOCK_SIZE - 1) / BLOCK_SIZE;
    initRangesKernel<<<num_blocks_init, BLOCK_SIZE>>>(d_ranges, params.num_beams);
    err = cudaGetLastError();
    if (err != cudaSuccess) { cudaFree(d_ranges); return err; }

    // Launch kernel to generate flatscan data
    int num_blocks_gen = (num_input_points + BLOCK_SIZE - 1) / BLOCK_SIZE;
    generateFlatScanKernel<<<num_blocks_gen, BLOCK_SIZE>>>(
        d_input_cloud, num_input_points, params, d_ranges);
    err = cudaGetLastError();
    if (err != cudaSuccess) { cudaFree(d_ranges); return err; }

    *d_ranges_output = d_ranges;
    return cudaSuccess;
}
