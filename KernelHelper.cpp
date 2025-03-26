#include "KernelHelper.h"




// Initialize CUDA
void initializeCuda() {
    int deviceCount = 0;
    CHECK_CUDA_ERROR(cudaGetDeviceCount(&deviceCount));

    if (deviceCount == 0) {
        std::cerr << "No CUDA capable devices found" << std::endl;
        return;
    }

    int maxComputeDevice = 0;
    int maxComputeCapability = 0;

    for (int dev = 0; dev < deviceCount; ++dev) {
        cudaDeviceProp deviceProp;
        CHECK_CUDA_ERROR(cudaGetDeviceProperties(&deviceProp, dev));

        int computeCapability = deviceProp.major * 10 + deviceProp.minor;
        if (computeCapability > maxComputeCapability) {
            maxComputeCapability = computeCapability;
            maxComputeDevice = dev;
        }
    }

    CHECK_CUDA_ERROR(cudaSetDevice(maxComputeDevice));

    // Print device info
    cudaDeviceProp deviceProp;
    CHECK_CUDA_ERROR(cudaGetDeviceProperties(&deviceProp, maxComputeDevice));
    std::cout << "Using CUDA device [" << maxComputeDevice << "]: "
        << deviceProp.name << " (Compute "
        << deviceProp.major << "." << deviceProp.minor << ")" << std::endl;
}


void shutdownCuda() {
    CHECK_CUDA_ERROR(cudaDeviceReset());
}


// Memory management helpers
bool allocateCudaMemory(void** devPtr, size_t size) {
    cudaError_t err = cudaMalloc(devPtr, size);
    return (err == cudaSuccess);
}

bool freeCudaMemory(void* devPtr) {
    cudaError_t err = cudaFree(devPtr);
    return (err == cudaSuccess);
}

bool copyToDevice(void* dst, const void* src, size_t size) {
    cudaError_t err = cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice);
    return (err == cudaSuccess);
}

bool copyFromDevice(void* dst, const void* src, size_t size) {
    cudaError_t err = cudaMemcpy(dst, src, size, cudaMemcpyDeviceToHost);
    return (err == cudaSuccess);
}


template<typename T>
bool copy3DArrayToDevice(T* devPtr, const std::vector<std::vector<std::vector<T>>>& hostArray, int N) {
    T* flatArray = new T[N * N * N];

    for (int x = 0; x < N; ++x) {
        for (int y = 0; y < N; ++y) {
            for (int z = 0; z < N; ++z) {
                flatArray[x * N * N + y * N + z] = hostArray[x][y][z];
            }
        }
    }

    bool result = copyToDevice(devPtr, flatArray, N * N * N * sizeof(T));
    delete[] flatArray;

    return result;
}

template<typename T>
bool copy3DArrayFromDevice(std::vector<std::vector<std::vector<T>>>& hostArray, const T* devPtr, int N) {
    T* flatArray = new T[N * N * N];

    bool result = copyFromDevice(flatArray, devPtr, N * N * N * sizeof(T));

    for (int x = 0; x < N; ++x) {
        for (int y = 0; y < N; ++y) {
            for (int z = 0; z < N; ++z) {
                hostArray[x][y][z] = flatArray[x * N * N + y * N + z];
            }
        }
    }

    delete[] flatArray;
    return result;
}

template bool copy3DArrayToDevice<bool>(bool* devPtr, const std::vector<std::vector<std::vector<bool>>>& hostArray, int N);
template bool copy3DArrayFromDevice<bool>(std::vector<std::vector<std::vector<bool>>>& hostArray, const bool* devPtr, int N);
template bool copy3DArrayToDevice<float>(float* devPtr, const std::vector<std::vector<std::vector<float>>>& hostArray, int N);
template bool copy3DArrayFromDevice<float>(std::vector<std::vector<std::vector<float>>>& hostArray, const float* devPtr, int N);
template bool copy3DArrayToDevice<double>(double* devPtr, const std::vector<std::vector<std::vector<double>>>& hostArray, int N);
template bool copy3DArrayFromDevice<double>(std::vector<std::vector<std::vector<double>>>& hostArray, const double* devPtr, int N);