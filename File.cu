#include "Header.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>

constexpr double EPSILON = 1e-6;

__device__ bool insideTriangleYZ(
    double y0, double z0,
    double y1, double z1,
    double y2, double z2,
    double y, double z)
{
    double w0 = (y - y0) * (z1 - z0) - (z - z0) * (y1 - y0);
    double w1 = (y - y1) * (z2 - z1) - (z - z1) * (y2 - y1);
    double w2 = (y - y2) * (z0 - z2) - (z - z2) * (y0 - y2);
    return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

__device__ bool intersectTriangleX(
    double x0, double y0, double z0,
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double y, double z,
    double& xi)
{
    double A = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0);
    if (fabs(A) < EPSILON) return false;
    double B = (z1 - z0) * (x2 - x0) - (x1 - x0) * (z2 - z0);
    double C = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    double D = -(A * x0 + B * y0 + C * z0);
    xi = (-D - B * y - C * z) / A;
    return true;
}

__global__ void voxelizeKernel(
    unsigned char* G, int res,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ,
    int numTris,
    const double* v0x, const double* v0y, const double* v0z,
    const double* v1x, const double* v1y, const double* v1z,
    const double* v2x, const double* v2y, const double* v2z)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    if (x >= res || y >= res || z >= res) return;

    double fx = (maxX - minX) / (res - 1.0);
    double fy = (maxY - minY) / (res - 1.0);
    double fz = (maxZ - minZ) / (res - 1.0);
    double vx = minX + x * fx + fx * 0.5;
    double vy = minY + y * fy + fy * 0.5;
    double vz = minZ + z * fz + fz * 0.5;

    int cnt = 0;
    for (int t = 0; t < numTris; ++t) {
        if (!insideTriangleYZ(v0y[t], v0z[t], v1y[t], v1z[t], v2y[t], v2z[t], vy, vz)) continue;
        double xi;
        if (!intersectTriangleX(v0x[t], v0y[t], v0z[t], v1x[t], v1y[t], v1z[t], v2x[t], v2y[t], v2z[t], vy, vz, xi)) continue;
        if (vx >= xi) ++cnt;
    }
    size_t idx = (x * res + y) * res + z;
    G[idx] = (cnt & 1) ? 1 : 0;
}

extern "C" void voxelizeOnGpu(
    const double* v0x, const double* v0y, const double* v0z,
    const double* v1x, const double* v1y, const double* v1z,
    const double* v2x, const double* v2y, const double* v2z,
    int numTris, int res,
    double minX, double maxX,
    double minY, double maxY,
    double minZ, double maxZ,
    unsigned char* outG)
{
    size_t triB = numTris * sizeof(double);
    size_t voxB = size_t(res) * res * res * sizeof(unsigned char);

    double* d0x, * d0y, * d0z, * d1x, * d1y, * d1z, * d2x, * d2y, * d2z;
    cudaMalloc(&d0x, triB); cudaMalloc(&d0y, triB); cudaMalloc(&d0z, triB);
    cudaMalloc(&d1x, triB); cudaMalloc(&d1y, triB); cudaMalloc(&d1z, triB);
    cudaMalloc(&d2x, triB); cudaMalloc(&d2y, triB); cudaMalloc(&d2z, triB);
    cudaMemcpy(d0x, v0x, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d0y, v0y, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d0z, v0z, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d1x, v1x, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d1y, v1y, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d1z, v1z, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d2x, v2x, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d2y, v2y, triB, cudaMemcpyHostToDevice);
    cudaMemcpy(d2z, v2z, triB, cudaMemcpyHostToDevice);

    unsigned char* dG;
    cudaMalloc(&dG, voxB);

    dim3 blk(8, 8, 8), grd((res + 7) / 8, (res + 7) / 8, (res + 7) / 8);
    voxelizeKernel << <grd, blk >> > (dG, res, minX, maxX, minY, maxY, minZ, maxZ, numTris,
        d0x, d0y, d0z, d1x, d1y, d1z, d2x, d2y, d2z);
    cudaDeviceSynchronize();

    cudaMemcpy(outG, dG, voxB, cudaMemcpyDeviceToHost);

    cudaFree(dG);
    cudaFree(d0x); cudaFree(d0y); cudaFree(d0z);
    cudaFree(d1x); cudaFree(d1y); cudaFree(d1z);
    cudaFree(d2x); cudaFree(d2y); cudaFree(d2z);
}