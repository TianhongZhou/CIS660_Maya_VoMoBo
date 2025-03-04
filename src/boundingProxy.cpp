#include "boundingProxy.h"

BoundingProxy::BoundingProxy() 
    : meshFn(nullptr)
{};

BoundingProxy::~BoundingProxy() { 
    delete meshFn; 
}

void* BoundingProxy::creator() {
    return new BoundingProxy();
}

MStatus BoundingProxy::doIt(const MArgList& args) {
    if (args.length() == 0) {
        MGlobal::displayWarning("No command argument received!");
        return MS::kFailure;
    }

    MString command = args.asString(0);

    if (command == "generate") {
        int resolution = args.asInt(1);
        SelectMesh();
        Voxelization(resolution);
        // closing
        // meshing
    }
    else if (command == "scale_field") {
        // beta version
    }
    else if (command == "show_voxel") {
        int resolution = args.asInt(1);
        SelectMesh();
        Voxelization(resolution);
        ShowVoxel(resolution);
    }
    else {
        MGlobal::displayWarning("Unknown command argument!");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

int BoundingProxy::World2Voxel(double w, double min, double max, int res) {
    int result = (w - min) / (max - min) * (res - 1);
    result = std::max(result, 0);
    result = std::min(result, res - 1);
    return result;
}

double BoundingProxy::Voxel2World(int v, double min, double max, int res) {
    return min + (v / (double) (res - 1)) * (max - min);
}

MVector BoundingProxy::CrossProduct(MVector a, MVector b) {
    return MVector(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

bool BoundingProxy::PlaneBoxOverlap(MPoint p, MVector n, MPoint v0) {
    // Get critical point c
    MPoint c;
    c.x = n.x > 0 ? deltaP.x : 0;
    c.y = n.y > 0 ? deltaP.y : 0;
    c.z = n.z > 0 ? deltaP.z : 0;

    // Determin T's plane overlaps B
    double np = n * p;
    double d1 = n * (c - v0);
    double d2 = n * (deltaP - c - v0);

    return (np + d1) * (np + d2) <= 0;
}

bool BoundingProxy::ProjectionBoxOverlap(int axis1, int axis2, int axis3, vector<MPoint> v, MVector n, MPoint p) {
    // Verify that T's aabb overlap B
    double triMinA = std::min({v[0][axis1], v[1][axis1] , v[2][axis1]});
    double triMaxA = std::max({v[0][axis1], v[1][axis1] , v[2][axis1]});
    double triMinB = std::min({v[0][axis2], v[1][axis2] , v[2][axis2]});
    double triMaxB = std::max({v[0][axis2], v[1][axis2] , v[2][axis2]});

    double boxMinA = p[axis1];
    double boxMaxA = p[axis1] + deltaP[axis1];
    double boxMinB = p[axis2];
    double boxMaxB = p[axis2] + deltaP[axis2];

    if (triMaxA < boxMinA || triMinA > boxMaxA ||
        triMaxB < boxMinB || triMinB > boxMaxB) {
        return false;
    }

    // And_{i=0}^2(dot(n_{e_i}^{ab}, p_{ab}) + d_{e_i}^{ab} >=0)
    for (int i = 0; i < 3; i++) {
        // e_i
        MVector ei = v[(i + 1) % 3] - v[i];

        // n_{e_i}^{xy}
        MVector n_ei = MVector(-ei[axis2], ei[axis1]);
        n_ei *= n[axis3] >= 0 ? 1 : -1;

        // p_{ab}
        MPoint p_ab = MPoint(p[axis1], p[axis2]);

        // d_{e_i}^{ab}
        MPoint vi_ab = MPoint(v[i][axis1], v[i][axis2]);
        double d_ei = -1 * n_ei * vi_ab + 
                        std::max(0.0, deltaP[axis1] * n_ei[0]) + 
                        std::max(0.0, deltaP[axis2] * n_ei[1]);

        if (n_ei * p_ab + d_ei < 0) {
            return false;
        }
    }
    return true;
}

bool BoundingProxy::TriangleBoxOverlap(MPoint voxel, MPoint v0, MPoint v1, MPoint v2) {
    // Get normal n of triangle T
    MVector e0 = v1 - v0;
    MVector e1 = v2 - v1;
    MVector e2 = v0 - v2;

    MVector n = CrossProduct(e0, e1);
    n.normalize();

    vector<MPoint> v = {v0, v1, v2};
    return (PlaneBoxOverlap(voxel, n, v0) &&
            ProjectionBoxOverlap(0, 1, 2, v, n, voxel) &&
            ProjectionBoxOverlap(1, 2, 0, v, n, voxel) &&
            ProjectionBoxOverlap(2, 0, 1, v, n, voxel));
}

// Method based-on "Fast Parallel Surface and Solid Voxelization on GPUs" (Schwarz, Seidel)
void BoundingProxy::Voxelization(int res) {
    // Initialize voxels G
    G = vector<vector<vector<bool>>>(res, vector<vector<bool>>(res, vector<bool>(res, false)));

    // Get vertices and triangles from meshFn
    MPointArray vertices;
    meshFn->getPoints(vertices, MSpace::kWorld);

    MIntArray triangleCounts, triangleIndices;
    meshFn->getTriangles(triangleCounts, triangleIndices);

    // Get bouding box of meshFn
    MPoint min, max;
    min = meshFn->boundingBox().min();
    max = meshFn->boundingBox().max();

    double minX = min.x, minY = min.y, minZ = min.z;
    double maxX = max.x, maxY = max.y, maxZ = max.z;
    deltaP = MPoint((maxX - minX) / res, (maxY - minY) / res, (maxZ - minZ) / res);

    // Iterate through all triangles
    for (int i = 0; i < triangleIndices.length(); i += 3) {
        MPoint v0 = vertices[triangleIndices[i]];
        MPoint v1 = vertices[triangleIndices[i + 1]];
        MPoint v2 = vertices[triangleIndices[i + 2]];

        // Convert to voxel space
        int x0 = World2Voxel(v0.x, minX, maxX, res);
        int y0 = World2Voxel(v0.y, minY, maxY, res);
        int z0 = World2Voxel(v0.z, minZ, maxZ, res);

        int x1 = World2Voxel(v1.x, minX, maxX, res);
        int y1 = World2Voxel(v1.y, minY, maxY, res);
        int z1 = World2Voxel(v1.z, minZ, maxZ, res);

        int x2 = World2Voxel(v2.x, minX, maxX, res);
        int y2 = World2Voxel(v2.y, minY, maxY, res);
        int z2 = World2Voxel(v2.z, minZ, maxZ, res);

        // Compute bounding box in voxel space
        int minX_vox = std::min({x0, x1, x2});
        int maxX_vox = std::max({x0, x1, x2});
        int minY_vox = std::min({y0, y1, y2});
        int maxY_vox = std::max({y0, y1, y2});
        int minZ_vox = std::min({z0, z1, z2});
        int maxZ_vox = std::max({z0, z1, z2});

        // Iterate over all voxels in this bounding box
        for (int x = minX_vox; x <= maxX_vox; x++) {
            for (int y = minY_vox; y <= maxY_vox; y++) {
                for (int z = minZ_vox; z <= maxZ_vox; z++) {
                    // Convert voxel center to world space
                    double vx = Voxel2World(x, minX, maxX, res);
                    double vy = Voxel2World(y, minY, maxY, res);
                    double vz = Voxel2World(z, minZ, maxZ, res);

                    MPoint voxelMinCorner(vx, vy, vz);

                    // Perform triangle/box overlap test
                    bool voxelStatus = TriangleBoxOverlap(voxelMinCorner, v0, v1, v2);
                    if (voxelStatus) {
                        G[x][y][z] = true;
                    }
                }
            }
        }
    }
}

MStatus BoundingProxy::SelectMesh() {
    MSelectionList selection;
    MGlobal::getActiveSelectionList(selection);

    MStatus status;
    status = selection.getDagPath(0, meshPath);

    if (!meshPath.hasFn(MFn::kMesh)) {
        MGlobal::displayWarning("Selected object is not a mesh!");
        return MS::kFailure;
    }

    delete meshFn;
    meshFn = new MFnMesh(meshPath);
}

void BoundingProxy::ShowVoxel(int res) {
    MString deleteOldVoxels = "if (`objExists voxelGroup`) { delete voxelGroup; } group -n voxelGroup;";
    MGlobal::executeCommand(deleteOldVoxels);

    int voxelCount = 0;

    for (int x = 0; x < res; x++) {
        for (int y = 0; y < res; y++) {
            for (int z = 0; z < res; z++) {
                if (G[x][y][z]) {
                    // Compute world space position of voxel
                    double px = x * deltaP.x;
                    double py = y * deltaP.y;
                    double pz = z * deltaP.z;

                    // Create a cube for the voxel
                    std::string cubeName = "voxelCube_" + std::to_string(voxelCount);
                    std::string cubeCmd = "polyCube -w " + std::to_string(deltaP.x) +
                        " -h " + std::to_string(deltaP.y) +
                        " -d " + std::to_string(deltaP.z) +
                        " -n " + cubeName + ";";
                    MGlobal::executeCommand(MString(cubeCmd.c_str()));

                    // Move the cube to the voxel position
                    std::string moveCmd = "move " + std::to_string(px) + " " +
                        std::to_string(py) + " " +
                        std::to_string(pz) + " " + cubeName + ";";
                    MGlobal::executeCommand(MString(moveCmd.c_str()));

                    // Parent to group
                    std::string parentCmd = "parent " + cubeName + " voxelGroup;";
                    MGlobal::executeCommand(MString(parentCmd.c_str()));

                    voxelCount++;
                }
            }
        }
    }
}