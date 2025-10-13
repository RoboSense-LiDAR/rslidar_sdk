#include "cuda_roi_filter.cuh"
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/execution_policy.h>

// Predicate for thrust::copy_if for ROI filtering
struct RoiFilterPredicate
{
    CudaRoiFilterConfig* d_filter_configs;
    size_t num_filter_configs;

    __host__ __device__
    RoiFilterPredicate(CudaRoiFilterConfig* filters, size_t num_filters)
        : d_filter_configs(filters), num_filter_configs(num_filters) {}

    __device__ bool operator()(const CudaPointXYZI& p) const
    {
        bool in_any_positive_box = false;
        bool in_any_negative_box = false;

        for (size_t i = 0; i < num_filter_configs; ++i)
        {
            CudaRoiFilterConfig filter_config = d_filter_configs[i];
            bool in_box = (p.x >= filter_config.min_x && p.x <= filter_config.max_x &&
                           p.y >= filter_config.min_y && p.y <= filter_config.max_y &&
                           p.z >= filter_config.min_z && p.z <= filter_config.max_z);

            if (in_box)
            {
                if (filter_config.type == 0) // Positive
                {
                    in_any_positive_box = true;
                }
                else if (filter_config.type == 1) // Negative
                {
                    in_any_negative_box = true;
                }
            }
        }

        // Priority logic: If in any positive box, keep. Else if not in any negative box, keep.
        return in_any_positive_box || (!in_any_negative_box && !in_any_positive_box);
    }
};

// Host-side function to launch the CUDA kernel for ROI filtering
cudaError_t roiFilterGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    const CudaRoiFilterConfig* h_filter_configs, // Host-side array of filter configs
    size_t num_filter_configs,
    CudaPointXYZI** d_output_cloud,
    size_t* num_output_points)
{
    if (num_input_points == 0 || num_filter_configs == 0)
    {
        *d_output_cloud = nullptr;
        *num_output_points = 0;
        return cudaSuccess;
    }

    // Copy filter configs to device
    CudaRoiFilterConfig* d_filter_configs;
    cudaError_t err = cudaMalloc((void**)&d_filter_configs, num_filter_configs * sizeof(CudaRoiFilterConfig));
    if (err != cudaSuccess) return err;
    err = cudaMemcpy(d_filter_configs, h_filter_configs, num_filter_configs * sizeof(CudaRoiFilterConfig), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { cudaFree(d_filter_configs); return err; }

    // Create predicate on device
    RoiFilterPredicate predicate(d_filter_configs, num_filter_configs);

    // Use thrust::copy_if to filter points
    thrust::device_ptr<CudaPointXYZI> d_input_ptr(d_input_cloud);
    
    // Pre-allocate output vector with the maximum possible size (same as input)
    thrust::device_vector<CudaPointXYZI> d_output_vec(num_input_points);

    // Perform the copy_if. The return value is an iterator to the end of the output range.
    auto new_end = thrust::copy_if(thrust::device, d_input_ptr, d_input_ptr + num_input_points,
                                   d_output_vec.begin(), predicate);

    // Get output size and copy result to output pointer
    *num_output_points = new_end - d_output_vec.begin();
    if (*num_output_points > 0)
    {
        // Resize the vector to the actual number of elements copied
        d_output_vec.resize(*num_output_points);

        err = cudaMalloc((void**)d_output_cloud, *num_output_points * sizeof(CudaPointXYZI));
        if (err != cudaSuccess) { cudaFree(d_filter_configs); return err; }
        
        // Copy the valid data from the resized vector to the final output pointer
        err = cudaMemcpy(*d_output_cloud, thrust::raw_pointer_cast(d_output_vec.data()), *num_output_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToDevice);
        if (err != cudaSuccess) { cudaFree(d_filter_configs); cudaFree(*d_output_cloud); return err; }
    }
    else
    {
        *d_output_cloud = nullptr;
    }

    cudaFree(d_filter_configs);
    return cudaSuccess;
}