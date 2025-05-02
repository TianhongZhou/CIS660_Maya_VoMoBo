#include "Header.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_atomic_functions.h> 
#include <algorithm>

using std::min;
using std::max;

constexpr double EPS = 1e-6;


__device__ int clamp_idx(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// YZ‐plane inside‐triangle test using POD Triangle
__device__ bool insideYZ(const GpuTriangle& T, double y, double z) {
    double y0 = T.v0y, z0 = T.v0z;
    double y1 = T.v1y, z1 = T.v1z;
    double y2 = T.v2y, z2 = T.v2z;
    double w0 = (y - y0) * (z1 - z0) - (z - z0) * (y1 - y0);
    double w1 = (y - y1) * (z2 - z1) - (z - z1) * (y2 - y1);
    double w2 = (y - y2) * (z0 - z2) - (z - z2) * (y0 - y2);
    return (w0 >= 0 && w1 >= 0 && w2 >= 0)
        || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

// intersect triangle plane with ray at (y,z) → x
__device__ bool intersectX(const GpuTriangle& T, double y, double z, double& xi) {
    double x0 = T.v0x, y0 = T.v0y, z0 = T.v0z;
    double x1 = T.v1x, y1 = T.v1y, z1 = T.v1z;
    double x2 = T.v2x, y2 = T.v2y, z2 = T.v2z;
    double A = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0);
    if (fabs(A) < EPS) return false;
    double B = (z1 - z0) * (x2 - x0) - (x1 - x0) * (z2 - z0);
    double C = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    double D = -(A * x0 + B * y0 + C * z0);
    xi = (-D - B * y - C * z) / A;
    return true;
}

// One CUDA thread per triangle
extern "C" __global__ void triangleKernelAoS(
    unsigned int* grid,
    int              res,
    double           minX, double maxX,
    double           minY, double maxY,
    double           minZ, double maxZ,
    int              numTris,
    const GpuTriangle* tris)
{
    int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= numTris) return;

    // load triangle
    GpuTriangle T = tris[t];

    // grid steps
    double fy = (maxY - minY) / (res - 1.0);
    double fz = (maxZ - minZ) / (res - 1.0);
    double fx = (maxX - minX) / (res - 1.0);

    // compute YZ AABB in voxel indices
    int yv0 = clamp_idx(int((T.v0y - minY) / fy), 0, res - 1);
    int yv1 = clamp_idx(int((T.v1y - minY) / fy), 0, res - 1);
    int yv2 = clamp_idx(int((T.v2y - minY) / fy), 0, res - 1);
    int zv0 = clamp_idx(int((T.v0z - minZ) / fz), 0, res - 1);
    int zv1 = clamp_idx(int((T.v1z - minZ) / fz), 0, res - 1);
    int zv2 = clamp_idx(int((T.v2z - minZ) / fz), 0, res - 1);

    int ymin = min(min(yv0, yv1), yv2);
    int ymax = max(max(yv0, yv1), yv2);
    int zmin = min(min(zv0, zv1), zv2);
    int zmax = max(max(zv0, zv1), zv2);

    // rasterize this triangle's spans
    for (int yv = ymin; yv <= ymax; ++yv) {
        double yc = minY + yv * fy + 0.5 * fy;
        for (int zv = zmin; zv <= zmax; ++zv) {
            double zc = minZ + zv * fz + 0.5 * fz;
            if (!insideYZ(T, yc, zc)) continue;

            double xi;
            if (!intersectX(T, yc, zc, xi)) continue;
            int xv = clamp_idx(int((xi - minX) / fx), 0, res - 1);

            // flip parity from xv..res-1 via atomicAdd
            for (int xx = xv; xx < res; ++xx) {
                size_t idx = (xx * res + yv) * res + zv;
                atomicAdd(&grid[idx], 1u);
            }
        }
    }
}

// Host wrapper
extern "C" void rasterizeMeshOnGpuTriangleAoS(
    const GpuTriangle* tris,
    int              numTris,
    int              res,
    double           minX, double maxX,
    double           minY, double maxY,
    double           minZ, double maxZ,
    unsigned int* outGrid)
{
    size_t N = size_t(res) * res * res;
    unsigned int* d_grid = nullptr;
    GpuTriangle* d_tris = nullptr;

    // 1) allocate & clear the grid
    cudaMalloc(&d_grid, N * sizeof(unsigned int));
    cudaMemset(d_grid, 0, N * sizeof(unsigned int));

    // 2) upload the triangle array
    cudaMalloc(&d_tris, numTris * sizeof(GpuTriangle));
    cudaMemcpy(d_tris, tris, numTris * sizeof(GpuTriangle), cudaMemcpyHostToDevice);

    // 3) launch one thread per triangle
    int threads = 128;
    int blocks = (numTris + threads - 1) / threads;
    triangleKernelAoS << <blocks, threads >> > (
        d_grid, res,
        minX, maxX,
        minY, maxY,
        minZ, maxZ,
        numTris,
        d_tris
        );
    cudaDeviceSynchronize();

    // 4) copy back crossing counts
    cudaMemcpy(outGrid, d_grid, N * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    // 5) free GPU buffers
    cudaFree(d_grid);
    cudaFree(d_tris);
}