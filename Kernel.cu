#include "KernelHelper.h"


const int THREADS_PER_BLOCK = 8; // 8x8x8 = 512 threads per block


// Device functions
__device__ inline int device_min(int a, int b) {
    return a < b ? a : b;
}

__device__ inline int device_max(int a, int b) {
    return a > b ? a : b;
}

__device__ float3_t cross(const float3_t& a, const float3_t& b) {
    return float3_t(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__device__ float dot(const float3_t& a, const float3_t& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float distance2(const float3_t& a, const float3_t& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// Utility function to convert voxel to world coordinates
__device__ float voxel2World(int v, float min, float max, int res) {
    return min + (v / static_cast<float>(res - 1)) * (max - min);
}

// Utility function to convert world to voxel coordinates
__device__ int world2Voxel(float w, float min, float max, int res) {
    int result = static_cast<int>((w - min) / (max - min) * (res - 1));
    result = device_max(result, 0);
    result = device_min(result, res - 1);
    return result;
}

// Edge function for inside triangle test
__device__ float edgeFunction(const float3_t& a, const float3_t& b, const float3_t& p) {
    return (p.x - a.y) * (b.z - a.z) - (p.y - a.z) * (b.y - a.y);
}

// Test if point (y,z) is inside triangle in YZ plane
__device__ bool insideTriangleYZ(const float3_t& v0, const float3_t& v1, const float3_t& v2, float y, float z) {
    float3_t p(0, y, z);

    float w0 = edgeFunction(v0, v1, p);
    float w1 = edgeFunction(v1, v2, p);
    float w2 = edgeFunction(v2, v0, p);

    return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

// Find x-intersection with triangle for a given (y,z) point
__device__ float intersectTriangleX(const float3_t& v0, const float3_t& v1, const float3_t& v2, float y, float z) {
    float3_t edge1 = v1 - v0;
    float3_t edge2 = v2 - v0;
    float3_t normal = cross(edge1, edge2);

    if (fabsf(normal.x) < 1e-6f)
        return INFINITY;

    float d = -dot(normal, v0);

    return (-normal.y * y - normal.z * z - d) / normal.x;
}

// Voxelization kernel 
__global__ void voxelizationKernel(bool* grid, const float3_t* vertices, const int* triangleIndices,
    int numTriangles, float3_t min, float3_t max, float3_t delta, int resolution) {
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (y >= resolution || z >= resolution)
        return;

    float worldY = voxel2World(y, min.y, max.y, resolution) + delta.y / 2.0f;
    float worldZ = voxel2World(z, min.z, max.z, resolution) + delta.z / 2.0f;

    // For each triangle
    for (int tri = 0; tri < numTriangles; tri++) {
        int idx0 = triangleIndices[tri * 3];
        int idx1 = triangleIndices[tri * 3 + 1];
        int idx2 = triangleIndices[tri * 3 + 2];

        float3_t v0 = vertices[idx0];
        float3_t v1 = vertices[idx1];
        float3_t v2 = vertices[idx2];

        int y0 = world2Voxel(v0.y, min.y, max.y, resolution);
        int z0 = world2Voxel(v0.z, min.z, max.z, resolution);
        int y1 = world2Voxel(v1.y, min.y, max.y, resolution);
        int z1 = world2Voxel(v1.z, min.z, max.z, resolution);
        int y2 = world2Voxel(v2.y, min.y, max.y, resolution);
        int z2 = world2Voxel(v2.z, min.z, max.z, resolution);

        int minY_vox = device_min(device_min(y0, y1), y2);
        int maxY_vox = device_max(device_max(y0, y1), y2);
        int minZ_vox = device_min(device_min(z0, z1), z2);
        int maxZ_vox = device_max(device_max(z0, z1), z2);

        if (y < minY_vox || y > maxY_vox || z < minZ_vox || z > maxZ_vox)
            continue;

  
        if (!insideTriangleYZ(v0, v1, v2, worldY, worldZ))
            continue;

        float xIntersect = intersectTriangleX(v0, v1, v2, worldY, worldZ);
        if (isinf(xIntersect))
            continue;

        int xIntersectVox = world2Voxel(xIntersect, min.x, max.x, resolution);

        for (int x = xIntersectVox; x < resolution; x++) {
            int idx = x * resolution * resolution + y * resolution + z;
            grid[idx] = !grid[idx];
        }
    }
}

// Dilation kernel for Algorithm 1
__global__ void dilationKernel(bool* output, const bool* input, const bool* gHat,
    const float* scaleField, float3_t delta, int resolution, bool useSphereSE) {
    int px = blockIdx.x * blockDim.x + threadIdx.x;
    int py = blockIdx.y * blockDim.y + threadIdx.y;
    int pz = blockIdx.z * blockDim.z + threadIdx.z;

    if (px >= resolution || py >= resolution || pz >= resolution)
        return;

    int idx = px * resolution * resolution + py * resolution + pz;

    if (input[idx])
        return;

    float scale = scaleField[idx];
    float3_t p(px, py, pz);

    bool shouldDilate = false;

    if (useSphereSE) {
        float radiusSq = scale * scale;

        int radius = ceil(scale);
        int startX = device_max(0, px - radius);
        int endX = device_min(resolution - 1, px + radius);
        int startY = device_max(0, py - radius);
        int endY = device_min(resolution - 1, py + radius);
        int startZ = device_max(0, pz - radius);
        int endZ = device_min(resolution - 1, pz + radius);

        for (int x = startX; x <= endX && !shouldDilate; x++) {
            for (int y = startY; y <= endY && !shouldDilate; y++) {
                for (int z = startZ; z <= endZ && !shouldDilate; z++) {
                    int neighborIdx = x * resolution * resolution + y * resolution + z;

                    if (gHat[neighborIdx]) {
                        float distSq = (x - px) * (x - px) + (y - py) * (y - py) + (z - pz) * (z - pz);
                        if (distSq <= radiusSq) {
                            shouldDilate = true;
                        }
                    }
                }
            }
        }
    }
    else {
        int radius = ceil(scale);
        int startX = device_max(0, px - radius);
        int endX = device_min(resolution - 1, px + radius);
        int startY = device_max(0, py - radius);
        int endY = device_min(resolution - 1, py + radius);
        int startZ = device_max(0, pz - radius);
        int endZ = device_min(resolution - 1, pz + radius);

        for (int x = startX; x <= endX && !shouldDilate; x++) {
            for (int y = startY; y <= endY && !shouldDilate; y++) {
                for (int z = startZ; z <= endZ && !shouldDilate; z++) {
                    int neighborIdx = x * resolution * resolution + y * resolution + z;
                    if (gHat[neighborIdx]) {
                        shouldDilate = true;
                    }
                }
            }
        }
    }

    output[idx] = shouldDilate;
}

// Connected contour kernel
__global__ void connectedContourKernel(bool* output, const bool* input, int resolution) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x <= 0 || y <= 0 || z <= 0 || x >= resolution - 1 || y >= resolution - 1 || z >= resolution - 1)
        return;

    int idx = x * resolution * resolution + y * resolution + z;

    if (!input[idx]) {
        output[idx] = false;
        return;
    }

    int dx[] = { 1, -1, 0, 0, 0, 0 };
    int dy[] = { 0, 0, 1, -1, 0, 0 };
    int dz[] = { 0, 0, 0, 0, 1, -1 };

    bool isContour = false;
    for (int i = 0; i < 6; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        int nz = z + dz[i];

        int nIdx = nx * resolution * resolution + ny * resolution + nz;
        if (!input[nIdx]) {
            isContour = true;
            break;
        }
    }

    output[idx] = isContour;
}

// Erosion kernel for Algorithm 2
__global__ void erosionKernel(bool* output, const bool* input, const bool* g,
    const bool* dcHat, const float* scaleField, int resolution) {
    int px = blockIdx.x * blockDim.x + threadIdx.x;
    int py = blockIdx.y * blockDim.y + threadIdx.y;
    int pz = blockIdx.z * blockDim.z + threadIdx.z;

    if (px >= resolution || py >= resolution || pz >= resolution)
        return;

    int idx = px * resolution * resolution + py * resolution + pz;

    if (!input[idx] || g[idx]) {
        output[idx] = input[idx];
        return;
    }

    bool shouldErode = false;
    float scale = scaleField[idx];
    float3_t p(px, py, pz);

    // Sphere test
    int radius = ceil(scale);
    int startX = device_max(0, px - radius);
    int endX = device_min(resolution - 1, px + radius);
    int startY = device_max(0, py - radius);
    int endY = device_min(resolution - 1, py + radius);
    int startZ = device_max(0, pz - radius);
    int endZ = device_min(resolution - 1, pz + radius);

    for (int x = startX; x <= endX && !shouldErode; x++) {
        for (int y = startY; y <= endY && !shouldErode; y++) {
            for (int z = startZ; z <= endZ && !shouldErode; z++) {
                int neighborIdx = x * resolution * resolution + y * resolution + z;

                if (dcHat[neighborIdx]) {
                    float distSq = (x - px) * (x - px) + (y - py) * (y - py) + (z - pz) * (z - pz);
                    float neighborScale = scaleField[neighborIdx];

                    if (distSq <= neighborScale * neighborScale) {
                        shouldErode = true;
                    }
                }
            }
        }
    }

    output[idx] = !shouldErode && input[idx];
}

// Pyramid calculation kernel
__global__ void pyramidKernel(bool* output, const bool* input, int inputResolution) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    int outputResolution = inputResolution / 2;

    if (x >= outputResolution || y >= outputResolution || z >= outputResolution)
        return;

    int outputIdx = x * outputResolution * outputResolution + y * outputResolution + z;

    bool result = false;
    for (int dx = 0; dx < 2; dx++) {
        for (int dy = 0; dy < 2; dy++) {
            for (int dz = 0; dz < 2; dz++) {
                int inputX = 2 * x + dx;
                int inputY = 2 * y + dy;
                int inputZ = 2 * z + dz;

                int inputIdx = inputX * inputResolution * inputResolution +
                    inputY * inputResolution + inputZ;

                result = result || input[inputIdx];
            }
        }
    }

    output[outputIdx] = result;
}

// Kernel launch functions
void launchVoxelizationKernel(bool* d_grid, float3_t* d_vertices, int* d_indices,
    int numTriangles, float3_t min, float3_t max, float3_t delta, int resolution) {
    dim3 blockSize(1, THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    dim3 gridSize(1, (resolution + blockSize.y - 1) / blockSize.y,
        (resolution + blockSize.z - 1) / blockSize.z);

    voxelizationKernel << <gridSize, blockSize >> > (d_grid, d_vertices, d_indices,
        numTriangles, min, max, delta, resolution);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void launchDilationKernel(bool* d_output, const bool* d_input, const bool* d_gHat,
    const float* d_scaleField, float3_t delta, int resolution, bool useSphereSE) {
    dim3 blockSize(THREADS_PER_BLOCK, THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    dim3 gridSize((resolution + blockSize.x - 1) / blockSize.x,
        (resolution + blockSize.y - 1) / blockSize.y,
        (resolution + blockSize.z - 1) / blockSize.z);

    dilationKernel << <gridSize, blockSize >> > (d_output, d_input, d_gHat, d_scaleField,
        delta, resolution, useSphereSE);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void launchConnectedContourKernel(bool* d_output, const bool* d_input, int resolution) {
    dim3 blockSize(THREADS_PER_BLOCK, THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    dim3 gridSize((resolution + blockSize.x - 1) / blockSize.x,
        (resolution + blockSize.y - 1) / blockSize.y,
        (resolution + blockSize.z - 1) / blockSize.z);

    connectedContourKernel << <gridSize, blockSize >> > (d_output, d_input, resolution);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void launchErosionKernel(bool* d_output, const bool* d_input, const bool* d_g,
    const bool* d_dcHat, const float* d_scaleField, int resolution) {
    dim3 blockSize(THREADS_PER_BLOCK, THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    dim3 gridSize((resolution + blockSize.x - 1) / blockSize.x,
        (resolution + blockSize.y - 1) / blockSize.y,
        (resolution + blockSize.z - 1) / blockSize.z);

    erosionKernel << <gridSize, blockSize >> > (d_output, d_input, d_g, d_dcHat, d_scaleField, resolution);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void launchPyramidKernel(bool* d_output, const bool* d_input, int inputResolution) {
    int outputResolution = inputResolution / 2;

    dim3 blockSize(THREADS_PER_BLOCK, THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    dim3 gridSize((outputResolution + blockSize.x - 1) / blockSize.x,
        (outputResolution + blockSize.y - 1) / blockSize.y,
        (outputResolution + blockSize.z - 1) / blockSize.z);

    pyramidKernel << <gridSize, blockSize >> > (d_output, d_input, inputResolution);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}