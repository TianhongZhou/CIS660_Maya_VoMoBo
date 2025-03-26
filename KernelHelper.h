#ifndef CUDA_HELPER_H
#define CUDA_HELPER_H


// Now include CUDA headers after handling Maya types
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <vector>
#include <iostream>

// Error checking macro for CUDA calls
#define CHECK_CUDA_ERROR(call) \
    { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << " at line " << __LINE__ << ": " \
                      << cudaGetErrorString(err) << " (" << err << ")" << std::endl; \
        } \
    }

// Struct to represent a 3D point in device code
struct float3_t {
    float x, y, z;

    __host__ __device__ float3_t() : x(0), y(0), z(0) {}
    __host__ __device__ float3_t(float x, float y, float z) : x(x), y(y), z(z) {}


    // Operators for CUDA device code
    __host__ __device__ float3_t operator+(const float3_t& other) const {
        return float3_t(x + other.x, y + other.y, z + other.z);
    }

    __host__ __device__ float3_t operator-(const float3_t& other) const {
        return float3_t(x - other.x, y - other.y, z - other.z);
    }

    __host__ __device__ float3_t operator*(float scalar) const {
        return float3_t(x * scalar, y * scalar, z * scalar);
    }
};

// CUDA device functions
__device__ float3_t cross(const float3_t& a, const float3_t& b);
__device__ float dot(const float3_t& a, const float3_t& b);
__device__ float distance2(const float3_t& a, const float3_t& b);

// Helper functions for memory management
bool allocateCudaMemory(void** devPtr, size_t size);
bool freeCudaMemory(void* devPtr);
bool copyToDevice(void* dst, const void* src, size_t size);
bool copyFromDevice(void* dst, const void* src, size_t size);

// Helper template to copy 3D arrays to/from device
template<typename T>
bool copy3DArrayToDevice(T* devPtr, const std::vector<std::vector<std::vector<T>>>& hostArray, int N);

template<typename T>
bool copy3DArrayFromDevice(std::vector<std::vector<std::vector<T>>>& hostArray, const T* devPtr, int N);

// CUDA kernel launch wrappers
void launchVoxelizationKernel(bool* d_grid, float3_t* d_vertices, int* d_indices,
    int numTriangles, float3_t min, float3_t max, float3_t delta, int resolution);

void launchDilationKernel(bool* d_output, const bool* d_input, const bool* d_gHat,
    const float* d_scaleField, float3_t delta, int resolution, bool useSphereSE);

void launchErosionKernel(bool* d_output, const bool* d_input, const bool* d_g,
    const bool* d_dcHat, const float* d_scaleField, int resolution);

void launchConnectedContourKernel(bool* d_output, const bool* d_input, int resolution);

void launchPyramidKernel(bool* d_output, const bool* d_input, int inputResolution);

// Utility functions
void initializeCuda();
void shutdownCuda();

#endif // CUDA_HELPER_H
