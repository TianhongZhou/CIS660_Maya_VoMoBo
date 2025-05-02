#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * A plain‐old‐data (POD) triangle.  No CUDA headers here,
     * so this can be included in your Maya plugin .cpp without any conflict.
     */
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

#ifdef __cplusplus
}
#endif