#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        double v0x, v0y, v0z;
        double v1x, v1y, v1z;
        double v2x, v2y, v2z;
    } GpuTriangle;

    void rasterizeMeshOnGpuTriangleAoS(
        const GpuTriangle* tris,
        int                 numTris,
        int                 res,
        double              minX, double maxX,
        double              minY, double maxY,
        double              minZ, double maxZ,
        unsigned int* outGrid
    );

    void computePyramidLevelOnGpu(
        const unsigned char* prev,
        unsigned char* curr,
        int                  NiPrev
    );

   
    void spatiallyVaryingDilationOnGpu(
        const unsigned char* G0_host,
        const unsigned char* GHat_packed,
        const double* S_host,
        unsigned char* D_host,
        int                  res,
        int                  iMax,
        double               dP,
        int                  useCube
    );

   

#ifdef __cplusplus
}
#endif