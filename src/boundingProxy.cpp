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

        // Iterate over yz voxel columns
        for (int y = minY_vox; y <= maxY_vox; y++) {
            for (int z = minZ_vox; z <= maxZ_vox; z++) {
                double vy = Voxel2World(y, minY, maxY, res);
                double vz = Voxel2World(z, minZ, maxZ, res);

                // Check if voxel column is inside the triangle projection
                if (!InsideTriangleYZ(v0, v1, v2, vy, vz)) continue;

                // Compute intersection with triangle plane
                double xIntersect = IntersectTriangleX(v0, v1, v2, vy, vz);
                int xIntersectVox = World2Voxel(xIntersect, minX, maxX, res);

                // Flip all voxels beyond the intersection
                for (int x = xIntersectVox; x < res; x++) {
                    G[x][y][z] = !G[x][y][z];  // XOR toggle
                }
            }
        }
    }
}

double BoundingProxy::EdgeFunction(MPoint a, MPoint b, MVector p) {
    return (p.x - a.y) * (b.z - a.z) - (p.y - a.z) * (b.y - a.y);
};

bool BoundingProxy::InsideTriangleYZ(MPoint v0, MPoint v1, MPoint v2, double y, double z) {
    MVector p(y, z);

    // Compute signed area tests (edge function tests)
    double w0 = EdgeFunction(v0, v1, p);
    double w1 = EdgeFunction(v1, v2, p);
    double w2 = EdgeFunction(v2, v0, p);

    return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

double BoundingProxy::IntersectTriangleX(MPoint v0, MPoint v1, MPoint v2, double y, double z) {
    // Triangle plane equation: Ax + By + Cz + D = 0
    MVector n_ei_yz = CrossProduct(v1 - v0, v2 - v0);
    double d_ei_yz = -n_ei_yz * v0;

    // Solve for x: x = (-D - By - Cz) / A
    if (fabs(n_ei_yz.x) < 1e-6) return std::numeric_limits<double>::max();

    return (-d_ei_yz - n_ei_yz.y * y - n_ei_yz.z * z) / n_ei_yz.x;
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