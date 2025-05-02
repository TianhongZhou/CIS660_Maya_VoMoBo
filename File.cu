#include "Header.h"           // must define GpuTriangle
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_atomic_functions.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>
#include <stack>

using std::min;
using std::max;

constexpr double EPS = 1e-6;


__device__ int clamp_idx(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

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

//triangle plane with ray at (y,z) → x
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

//per triangle
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

    GpuTriangle T = tris[t];

    double fy = (maxY - minY) / (res - 1.0);
    double fz = (maxZ - minZ) / (res - 1.0);
    double fx = (maxX - minX) / (res - 1.0);

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

    cudaMalloc(&d_grid, N * sizeof(unsigned int));
    cudaMemset(d_grid, 0, N * sizeof(unsigned int));

    cudaMalloc(&d_tris, numTris * sizeof(GpuTriangle));
    cudaMemcpy(d_tris, tris, numTris * sizeof(GpuTriangle), cudaMemcpyHostToDevice);

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

    cudaMemcpy(outGrid, d_grid, N * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    cudaFree(d_grid);
    cudaFree(d_tris);
}




extern "C" __global__ void pyramidKernel(
    const unsigned char* prev, unsigned char* curr, int NiPrev)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int Ni = NiPrev >> 1;
    int N = Ni * Ni * Ni;
    if (idx >= N)return;
    int x = idx / (Ni * Ni), y = (idx / Ni) % Ni, z = idx % Ni;
    unsigned char v = 0;
    int bx = x << 1, by = y << 1, bz = z << 1;
    for (int dx = 0; dx < 2; ++dx)
        for (int dy = 0; dy < 2; ++dy)
            for (int dz = 0; dz < 2; ++dz) {
                int p = ((bx + dx) * NiPrev + (by + dy)) * NiPrev + (bz + dz);
                v |= prev[p];
            }
    curr[idx] = v;
}

extern "C" void computePyramidLevelOnGpu(
    const unsigned char* prev, unsigned char* curr, int NiPrev)
{
    int Ni = NiPrev >> 1;
    size_t nelemsPrev = size_t(NiPrev) * NiPrev * NiPrev;
    size_t nelemsCurr = size_t(Ni) * Ni * Ni;
    size_t bytesPrev = nelemsPrev * sizeof(unsigned char);
    size_t bytesCurr = nelemsCurr * sizeof(unsigned char);
    unsigned char* dPrev = nullptr;
    unsigned char* dCurr = nullptr;
    cudaMalloc(&dPrev, bytesPrev);
    cudaMalloc(&dCurr, bytesCurr);
    cudaMemcpy(dPrev, prev, bytesPrev, cudaMemcpyHostToDevice);
    int threads = 256, blocks = (nelemsCurr + threads - 1) / threads;
    pyramidKernel << <blocks, threads >> > (dPrev, dCurr, NiPrev);
    cudaDeviceSynchronize();
    cudaMemcpy(curr, dCurr, bytesCurr, cudaMemcpyDeviceToHost);
    cudaFree(dPrev);
    cudaFree(dCurr);
}

constexpr int MAX_LEVEL = 11;  //11 for up to 2048³

__device__ __forceinline__ size_t levelOffset(int lvl, int res) {
    size_t off = 0;
    for (int L = 1; L < lvl; ++L) {
        int d = res >> L;
        off += size_t(d) * d * d;
    }
    return off;
}

__device__ __forceinline__ bool sampleGHat(
    const unsigned char* GHat,
    int lvl, int res,
    int cx, int cy, int cz)
{
    int d = res >> lvl;
    size_t off = levelOffset(lvl, res);
    size_t idx = off + size_t(cx) * d * d + size_t(cy) * d + size_t(cz);
    return GHat[idx] != 0;
}



extern "C" __global__
void spatiallyVaryingDilationKernel(
    const unsigned char* __restrict__ G0,
    const unsigned char* __restrict__ GHat,
    const double* __restrict__ S,
    unsigned char* __restrict__ D,
    int res,
    int iMax,
    double dP,
    int useCube
) {
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    size_t N = size_t(res) * res * res;
    if (idx >= N) return;

    // unravel voxel coords
    int x = idx / (res * res);
    int y = (idx / res) % res;
    int z = idx % res;

    // If already occupied
    if (G0[idx]) {
        D[idx] = 1;
        return;
    }

    // local stack of (level, qx, qy, qz)
    int stackLvl[MAX_LEVEL + 1];
    int stackQx[MAX_LEVEL + 1];
    int stackQy[MAX_LEVEL + 1];
    int stackQz[MAX_LEVEL + 1];
    int sp = 0;

    // push the top‐level cell
    stackLvl[0] = iMax;
    stackQx[0] = 0;
    stackQy[0] = 0;
    stackQz[0] = 0;

    // precompute world‐radius & voxel‐radius
    double rworld = S[idx];
    int    rvox = min(res - 1,
        max(0, int(ceil(rworld / dP))));

    bool hit = false;
    while (sp >= 0 && !hit) {
        int lvl = stackLvl[sp];
        int qx = stackQx[sp];
        int qy = stackQy[sp];
        int qz = stackQz[sp];
        --sp;

        if (lvl == 0) {
            hit = true;
            break;
        }

        int twoi = 1 << lvl;     
        double dxs = twoi;     
        double minx = double(qx) * dxs;
        double miny = double(qy) * dxs;
        double minz = double(qz) * dxs;
        double maxx = minx + dxs;
        double maxy = miny + dxs;
        double maxz = minz + dxs;

        // build axis‐aligned box [min*,max*] around sub‐cell
        double wx = x, wy = y, wz = z;
        double bminx = wx - rvox, bminy = wy - rvox, bminz = wz - rvox;
        double bmaxx = wx + rvox + 1, bmaxy = wy + rvox + 1, bmaxz = wz + rvox + 1;

        // AABB‐test:
        if (!(maxx >= bminx && minx <= bmaxx &&
            maxy >= bminy && miny <= bmaxy &&
            maxz >= bminz && minz <= bmaxz))
            continue;

        //for sphere testdo an extra distance check:
        bool passesShape = true;
        if (useCube == 0) {
            double cx = fmin(fmax(wx, minx), maxx);
            double cy = fmin(fmax(wy, miny), maxy);
            double cz = fmin(fmax(wz, minz), maxz);
            double dd = (wx - cx) * (wx - cx)
                + (wy - cy) * (wy - cy)
                + (wz - cz) * (wz - cz);
            passesShape = (dd <= rworld * rworld);
        }
        if (!passesShape) continue;

        int half = twoi >> 1;
        for (int rx = 0; rx < 2; ++rx) {
            for (int ry = 0; ry < 2; ++ry) {
                for (int rz = 0; rz < 2; ++rz) {
                    int cxq = qx * 2 + rx;
                    int cyq = qy * 2 + ry;
                    int czq = qz * 2 + rz;
                    if (sampleGHat(GHat, lvl - 1, res, cxq, cyq, czq)) {
                        ++sp;
                        stackLvl[sp] = lvl - 1;
                        stackQx[sp] = cxq;
                        stackQy[sp] = cyq;
                        stackQz[sp] = czq;
                    }
                }
            }
        }
    }

    D[idx] = hit ? 1 : 0;
}


// host wrapper
extern "C" void spatiallyVaryingDilationOnGpu(
    const unsigned char* G0_host,
    const unsigned char* GHat_packed,
    const double* S_host,
    unsigned char* D_host,
    int                  res,
    int                  iMax,
    double               dP,
    int                  useCube
) {
    size_t N = size_t(res) * res * res;
    // compute GH total size
    size_t ghTot = 0;
    for (int lvl = 1; lvl <= iMax; ++lvl) {
        int d = res >> lvl;
        ghTot += size_t(d) * d * d;
    }

    unsigned char* d_G0, * d_GH, * d_D;
    double* d_S;

    cudaMalloc(&d_G0, N);
    cudaMemcpy(d_G0, G0_host, N, cudaMemcpyHostToDevice);

    cudaMalloc(&d_GH, ghTot);
    cudaMemcpy(d_GH, GHat_packed, ghTot, cudaMemcpyHostToDevice);

    cudaMalloc(&d_S, N * sizeof(double));
    cudaMemcpy(d_S, S_host, N * sizeof(double), cudaMemcpyHostToDevice);

    cudaMalloc(&d_D, N);
    cudaMemset(d_D, 0, N);

    dim3 block(256), grid((N + 255) / 256);
    spatiallyVaryingDilationKernel << <grid, block >> > (
        d_G0, d_GH, d_S, d_D,
        res, iMax, dP, useCube
        );
    cudaDeviceSynchronize();

    cudaMemcpy(D_host, d_D, N, cudaMemcpyDeviceToHost);

    cudaFree(d_G0);
    cudaFree(d_GH);
    cudaFree(d_S);
    cudaFree(d_D);
}




