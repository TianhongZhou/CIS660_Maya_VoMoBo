#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * Runs GPU voxelization: allocates device memory, copies triangles,
     * launches kernel, copies back a byte grid [0/1], frees memory.
     * @param v0x..v2z host arrays of length numTris
     * @param numTris number of triangles
     * @param res      grid resolution (res^3 voxels)
     * @param minX..maxZ world bounds
     * @param outG     host buffer (unsigned char*) of length res^3
     */
    void voxelizeOnGpu(
        const double* v0x, const double* v0y, const double* v0z,
        const double* v1x, const double* v1y, const double* v1z,
        const double* v2x, const double* v2y, const double* v2z,
        int            numTris,
        int            res,
        double         minX, double maxX,
        double         minY, double maxY,
        double         minZ, double maxZ,
        unsigned char* outG
    );

#ifdef __cplusplus
}
#endif
