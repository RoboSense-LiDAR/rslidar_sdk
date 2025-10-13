#include "cuda_flatscan.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>
#include <limits.h> // For INT_MAX

// Kernel to initialize integer ranges array with INT_MAX
__global__ void initRangesKernelInt(int* d_ranges_mm, size_t num_beams)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_beams)
    {
        d_ranges_mm[idx] = INT_MAX;
    }
}

// Kernel to generate LaserScan data using integer atomics
__global__ void generateFlatScanKernelInt(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    CudaLaserScanParams params,
    int* d_ranges_mm)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_input_points) return;

    CudaPointXYZI p = d_input_cloud[idx];

    // Check height range
    if (p.z >= params.min_height && p.z <= params.max_height)
    {
        float range_m = sqrtf(p.x * p.x + p.y * p.y);

        // Check range limits
        if (range_m >= params.range_min && range_m <= params.range_max)
        {
            float angle = atan2f(p.y, p.x);
            // Calculate index for the angular bin
            int bin_idx = static_cast<int>((angle - params.angle_min) / params.angle_increment);

            if (bin_idx >= 0 && bin_idx < params.num_beams)
            {
                // Convert range to integer centimeters
                int range_cm = static_cast<int>(range_m * 1000.0f);
                // Use native integer atomicMin
                atomicMin(&d_ranges_mm[bin_idx], range_cm);
            }
        }
    }
}

// Kernel to convert integer ranges (cm) back to float (m)
__global__ void convertRangesToFloatKernel(int* d_ranges_mm, float* d_ranges_m, size_t num_beams)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_beams)
    {
        if (d_ranges_mm[idx] == INT_MAX)
        {
            d_ranges_m[idx] = INFINITY;
        }
        else
        {
            d_ranges_m[idx] = static_cast<float>(d_ranges_mm[idx]) / 1000.0f;
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

    // 1. Allocate intermediate integer GPU memory for ranges in cm
    int* d_ranges_mm;
    err = cudaMalloc((void**)&d_ranges_mm, params.num_beams * sizeof(int));
    if (err != cudaSuccess) return err;

    // 2. Allocate final float GPU memory for ranges in m
    float* d_ranges_m;
    err = cudaMalloc((void**)&d_ranges_m, params.num_beams * sizeof(float));
    if (err != cudaSuccess) 
    {
        cudaFree(d_ranges_mm);
        return err;
    }

    // Get optimal block size
    int min_grid_size;
    int block_size;
    cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size, generateFlatScanKernelInt, 0, 0);
    const int BLOCK_SIZE = block_size > 0 ? block_size : 256; // Fallback block size

    // 3. Initialize integer ranges to INT_MAX
    int num_blocks_init = (params.num_beams + BLOCK_SIZE - 1) / BLOCK_SIZE;
    initRangesKernelInt<<<num_blocks_init, BLOCK_SIZE>>>(d_ranges_mm, params.num_beams);
    err = cudaGetLastError();
    if (err != cudaSuccess) { cudaFree(d_ranges_mm); cudaFree(d_ranges_m); return err; }

    // 4. Launch kernel to generate flatscan data into the integer array
    int num_blocks_gen = (num_input_points + BLOCK_SIZE - 1) / BLOCK_SIZE;
    generateFlatScanKernelInt<<<num_blocks_gen, BLOCK_SIZE>>>(d_input_cloud, num_input_points, params, d_ranges_mm);
    err = cudaGetLastError();
    if (err != cudaSuccess) { cudaFree(d_ranges_mm); cudaFree(d_ranges_m); return err; }

    // 5. Launch kernel to convert integer (cm) ranges to float (m) ranges
    int num_blocks_convert = (params.num_beams + BLOCK_SIZE - 1) / BLOCK_SIZE;
    convertRangesToFloatKernel<<<num_blocks_convert, BLOCK_SIZE>>>(d_ranges_mm, d_ranges_m, params.num_beams);
    err = cudaGetLastError();
    if (err != cudaSuccess) { cudaFree(d_ranges_mm); cudaFree(d_ranges_m); return err; }

    // 6. Free the intermediate integer array
    cudaFree(d_ranges_mm);

    // 7. Set the output pointer and return
    *d_ranges_output = d_ranges_m;
    return cudaSuccess;
}