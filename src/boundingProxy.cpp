#include "boundingProxy.h"

BoundingProxy::BoundingProxy() 
    : meshFn(nullptr), editedS(false), voxelCount(0)
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

    if (command == "reset_scale_field") {
        ResetScaleField();
    } 
    else if (command == "generate") {
        int resolution = args.asInt(1);
        MString SE = args.asString(3);
        double baseScale = args.asDouble(4);
        meshName = args.asString(5);

        SelectMesh();

        if (args.asString(2) == "cpu") {
            // Compute G
            VoxelizationCPU(resolution);
            // Compute GHat
            PyramidGCPU();
            // Compute D
            DilationCPU(SE, baseScale);
            // Compute Dc
            ConnectedContourCPU();
            // Compute DcHat
            ScaleAugmentedPyramidCPU();
            // Compute E
            ErosionCPU();
            // Store E to outputG
            outputG = E;
            // Compute Mesh C
            CubeMarching();
            // Show Mesh C
            CreateMayaMesh("E");
        }
        else {
            // TODO: GPU
        }
    }
    // For debug purpose only
    else if (command == "show_voxel") {
        int resolution = args.asInt(1);
        MString SE = args.asString(3);
        double baseScale = args.asDouble(4);
        meshName = args.asString(5);

        SelectMesh();

        if (args.asString(2) == "cpu") {
            VoxelizationCPU(resolution);
            PyramidGCPU();
            DilationCPU(SE, baseScale);
            ConnectedContourCPU();
            ScaleAugmentedPyramidCPU();
            ErosionCPU();
        }
        else {
            // TODO: GPU
        }
        // Can show G, GHat[i], D, Dc, DcHat[i].first, E
        ShowVoxel(G, "G");
    }
    else {
        MGlobal::displayWarning("Unknown command argument!");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

// Convert Eigen V and F to Maya mesh
void BoundingProxy::CreateMayaMesh(MString name) {
    MPointArray mayaVertices;
    MIntArray mayaFaceCounts;
    MIntArray mayaFaceConnects;

    // Convert Eigen vertices to MPointArray
    for (int i = 0; i < V.rows(); i++) {
        mayaVertices.append(MPoint(V(i, 0), V(i, 1), V(i, 2)));
    }

    // Convert Eigen faces to MIntArray
    for (int i = 0; i < F.rows(); i++) {
        mayaFaceConnects.append(F(i, 0));
        mayaFaceConnects.append(F(i, 1));
        mayaFaceConnects.append(F(i, 2));
        mayaFaceCounts.append(3);
    }

    // Create the mesh in Maya
    MFnMesh meshFn;
    MObject newMesh = meshFn.create(
        mayaVertices.length(), 
        mayaFaceCounts.length(), 
        mayaVertices,          
        mayaFaceCounts,     
        mayaFaceConnects,    
        MObject::kNullObj    
    );

    // Name the mesh
    MFnDagNode dagNode(newMesh);
    dagNode.setName(name);
}

// Convert outputG to mesh using cube marchings from igl adn eigen
void BoundingProxy::CubeMarching() {
    int N = (int) outputG.size();
    int newN = N + 2;

    Eigen::MatrixXd Sm(newN * newN * newN, 1);
    Eigen::MatrixXd GV(newN * newN * newN, 3);

    double newDeltaX = deltaP.x * N / newN;
    double newDeltaY = deltaP.y * N / newN;
    double newDeltaZ = deltaP.z * N / newN;

    int index = 0;
    for (int x = 0; x < newN; x++) {
        for (int y = 0; y < newN; y++) {
            for (int z = 0; z < newN; z++) {
                if (x == 0 || y == 0 || z == 0 || x == newN - 1 || y == newN - 1 || z == newN - 1) {
                    Sm(index, 0) = 0.0;
                }
                else {
                    Sm(index, 0) = outputG[x - 1][y - 1][z - 1] ? 1.0 : 0.0;
                }

                GV(index, 0) = (x - 1) * newDeltaX;
                GV(index, 1) = (y - 1) * newDeltaY;
                GV(index, 2) = (z - 1) * newDeltaZ;

                index++;
            }
        }
    }

    igl::copyleft::marching_cubes(Sm, GV, newN, newN, newN, V, F);
}

// Algorithm 2 - Parallel spatially varying erosion
void BoundingProxy::ErosionCPU() {
    int Ni = (int) D.size();
    int iMax = (int) log2(Ni);

    // r in {0,1}^3
    vector<MPoint> r = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};

    // E <- D
    E = D;

    for (int px = 0; px < Ni; px++) {
        for (int py = 0; py < Ni; py++) {
            for (int pz = 0; pz < Ni; pz++) {
                // for all voxel p | E[p] != 0 && G[p] = 0 do
                if (!(E[px][py][pz] && !G[px][py][pz])) {
                    continue;
                }

                // T <- (iMax, 0)
                stack<pair<int, MPoint>> T;
                T.push(pair<int, MPoint>(iMax, MPoint(0, 0, 0)));

                // while T not empty and E[p] != 0 do
                while (!T.empty() && E[px][py][pz]) {
                    // v = (i, q) <- peek of T
                    pair<int, MPoint> v = T.top();
                    int i = v.first;
                    MPoint q = v.second;

                    // T <- T \ v
                    T.pop();

                    if (i == 0) {
                        // E[p] <- 0
                        E[px][py][pz] = false;
                    }
                    else {
                        // for all sub-cell vr = (i - 1, 2q + r) of v do
                        for (int k = 0; k < r.size(); k++) {
                            // vr = (i - 1, 2q + r)
                            pair<int, MPoint> vr = pair<int, MPoint>(i - 1, 2 * q + r[k]);
                            // vr = (i, q)
                            // DcHat[vr] = Dci[q] = (b, s)
                            bool b = DcHat[vr.first].first[(int) vr.second.x][(int) vr.second.y][(int) vr.second.z];
                            double s = DcHat[vr.first].second[(int) vr.second.x][(int) vr.second.y][(int) vr.second.z];

                            // vr_spatial_dilation N p_spatial
                            // vr_spatial = {q + y, y in [0, 2^i]^3}
                            // vr_spatial_dilation = {q + y, ||y|| < s}
                            // Need to convert q from Di space to D0 space before collision test
                            double twoi = pow(2, vr.first);
                            MPoint vrSpatialDilationMin = vr.second * twoi - MPoint(s, s, s);
                            MPoint vrSpatialDilationMax = vr.second * twoi + MPoint(s, s, s) + MPoint(twoi, twoi, twoi);

                            // p_spatial = {p + y, y in [0, 1]^3}
                            MPoint p = MPoint(px, py, pz);
                            MPoint pMin = p;
                            MPoint pMax = p + MPoint(1, 1, 1);

                            bool intersect = false;
                            // AABB collision test first
                            intersect = vrSpatialDilationMax.x >= pMin.x && vrSpatialDilationMin.x <= pMax.x &&
                                        vrSpatialDilationMax.y >= pMin.y && vrSpatialDilationMin.y <= pMax.y &&
                                        vrSpatialDilationMax.z >= pMin.z && vrSpatialDilationMin.z <= pMax.z;

                            if (!intersect || !b) {
                                continue;
                            }

                            // Round corner test secondly
                            intersect = false;
                            MPoint vrSpatialMin = vr.second * twoi;
                            MPoint vrSpatialMax = vr.second * twoi + MPoint(twoi, twoi, twoi);

                            for (int m = 0; m < r.size(); m++) {
                                MPoint pCorner = p + r[m];

                                // Find closest point between pCorner and vrSpatial
                                double closestx = max(vrSpatialMin.x, min(pCorner.x, vrSpatialMax.x));
                                double closesty = max(vrSpatialMin.y, min(pCorner.y, vrSpatialMax.y));
                                double closestz = max(vrSpatialMin.z, min(pCorner.z, vrSpatialMax.z));

                                double dSquare = pow(pCorner.x - closestx, 2) + pow(pCorner.y - closesty, 2) + pow(pCorner.z - closestz, 2);
                                intersect = dSquare <= s * s;
                                if (intersect) {
                                    break;
                                }
                            }

                            // if vr_spatial_dilation N p_spatial != empty and DcHat[vr] = (1,.)
                            if (intersect && b) {
                                // T <- T U vr
                                T.push(vr);
                            }
                        }
                    }
                }
            }
        }
    }
}

// Build a scale-augmented pyramid DcHat = {Dc^i} with Dc^i : N_i^3 -> {0,1} x R
void BoundingProxy::ScaleAugmentedPyramidCPU() {
    int N0 = (int) Dc.size();
    int iMax = (int) log2(N0);
    // Dc0[p] = (Dc[p], S[p]) 
    DcHat[0].first = Dc;
    DcHat[0].second = S;

    // t in {0,1}^3
    vector<MPoint> t = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};

    for (int i = 1; i <= iMax; i++) {
        DcHat[i].first = CalculatePyramid(DcHat[i - 1].first, t, [](bool a, bool b) {return a || b;});
        DcHat[i].second = CalculatePyramid(DcHat[i - 1].second, t, [](double a, double b) {return max(a, b);});
    }
}

// Compute Dc from D using 6-connected contour
void BoundingProxy::ConnectedContourCPU() {
    int N = (int) D.size();
    vector<MPoint> r = {{0,0,1}, {0,1,0}, {1,0,0}, {0,0,-1}, {0,-1,0}, {-1,0,0}};
    Dc = D;

    for (int x = 1; x < N - 1; x++) {
        for (int y = 1; y < N - 1; y++) {
            for (int z = 1; z < N - 1; z++) {
                if (D[x][y][z]) {
                    bool flag = true;
                    for (int i = 0; i < r.size(); i++) {
                        MPoint p = MPoint(x, y, z) + r[i];
                        if (!D[(int) p.x][(int) p.y][(int) p.z]) {
                            Dc[x][y][z] = true;
                            flag = false;
                            break;
                        }
                    }
                    if (flag) {
                        Dc[x][y][z] = false;
                    }
                }
            }
        }
    }
}

// Algorithm 1 - Parallel spatially varying dilation
void BoundingProxy::DilationCPU(MString SE, double baseScale) {
    int Ni = (int) G.size();
    if (!editedS) {
        // Initialize S if haven't
        S = vector<vector<vector<double>>>(Ni, vector<vector<double>>(Ni, vector<double>(Ni, baseScale)));
    }

    int iMax = (int) log2(Ni);

    // r in {0,1}^3
    vector<MPoint> r = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};

    // D <- G
    D = G;

    for (int px = 0; px < Ni; px++) {
        for (int py = 0; py < Ni; py++) {
            for (int pz = 0; pz < Ni; pz++) {
                // for all voxel p | D[p] = 0 do
                if (D[px][py][pz]) {
                    continue;
                }

                // T <- (iMax, 0)
                stack<pair<int, MPoint>> T;
                T.push(pair<int, MPoint>(iMax, MPoint(0, 0, 0)));

                // while T not empty and D[p] != 1 do
                while (!T.empty() && !D[px][py][pz]) {
                    // v = (i, q) <- peek of T
                    pair<int, MPoint> v = T.top();
                    int i = v.first;
                    MPoint q = v.second;

                    // T <- T \ v
                    T.pop();

                    if (i == 0) {
                        // D[p] <- 1
                        D[px][py][pz] = true;
                    }
                    else {
                        // for all sub-cell vr = (i - 1, 2q + r) of v do
                        for (int k = 0; k < r.size(); k++) {
                            // vr = (i - 1, 2q + r)
                            pair<int, MPoint> vr = pair<int, MPoint>(i - 1, 2 * q + r[k]);

                            // Bs(p) N vr_spatial
                            // v = (i, q) => v_spatial = {q + y, y in [0, 2^i]^3}
                            // Need to convert q from Gi space to G0 space before collision test
                            double twoi = pow(2, vr.first);
                            MPoint vrSpatialMin = vr.second * twoi;
                            MPoint vrSpatialMax = vr.second * twoi + MPoint(twoi, twoi, twoi);
                            
                            bool intersect = false;
                            double Sp = S[px][py][pz];
                            MPoint p = MPoint(px, py, pz);
                            if (SE == "cube") {
                                // AABB collision test for two cubes
                                // Bs(p) = {p + y, y in [-S[p], S[p]]^3}
                                MPoint BspMin = p - MPoint(Sp, Sp, Sp);
                                MPoint BspMax = p + MPoint(Sp, Sp, Sp);

                                intersect = vrSpatialMax.x >= BspMin.x && vrSpatialMin.x <= BspMax.x &&
                                            vrSpatialMax.y >= BspMin.y && vrSpatialMin.y <= BspMax.y &&
                                            vrSpatialMax.z >= BspMin.z && vrSpatialMin.z <= BspMax.z;
                            }
                            else {
                                // Cube and sphere collision
                                // Bs(p) = {p + y, ||y|| < S[p]}
                                double closestx = max(vrSpatialMin.x, min(p.x, vrSpatialMax.x));
                                double closesty = max(vrSpatialMin.y, min(p.y, vrSpatialMax.y));
                                double closestz = max(vrSpatialMin.z, min(p.z, vrSpatialMax.z));

                                double dSquare = pow(p.x - closestx, 2) + pow(p.y - closesty, 2) + pow(p.z - closestz, 2);
                                intersect = dSquare <= Sp * Sp;
                            }

                            // if Bs(p) N vr_spatial != empty and GHat[vr] = 1
                            if (intersect && GHat[vr.first][(int) vr.second.x][(int) vr.second.y][(int) vr.second.z] == 1) {
                                // T <- T U vr
                                T.push(vr);
                            }
                        }
                    }
                }
            }
        }
    }
}

void BoundingProxy::ResetScaleField() {
    editedS = false;
    S.clear();
}

// for all p in Ni^3, Gi[p] = max_t G^{i - 1}[2p + t];
// for all p in Ni^3, Dci[p] = max_t Dc^{i - 1}[2p + t];
template <typename T, typename CombineOp>
vector<vector<vector<T>>> BoundingProxy::CalculatePyramid(vector<vector<vector<T>>> prev, vector<MPoint> t, CombineOp combine) {
    int Ni_1 = (int) prev.size();
    int Ni = Ni_1 / 2;

    vector<vector<vector<T>>> curr(Ni, vector<vector<T>>(Ni, vector<T>(Ni, T{})));

    for (int px = 0; px < Ni; px++) {
        for (int py = 0; py < Ni; py++) {
            for (int pz = 0; pz < Ni; pz++) {
                for (int i = 0; i < t.size(); i++) {
                    MPoint ti = t[i];
                    MPoint p = MPoint(px, py, pz);
                    MPoint p2t = 2 * p + ti;

                    curr[px][py][pz] = combine(curr[px][py][pz], prev[(int) p2t.x][(int) p2t.y][(int) p2t.z]);
                }
            }
        }
    }
    return curr;
}

// Build pyramid G_hat = {Gi} with Gi : N_i^3 -> {0,1}, and Ni = N0 / 2^i
void BoundingProxy::PyramidGCPU() {
    int N0 = (int) G.size();
    int iMax = (int) log2(N0);
    // G0[p] = G[p] 
    GHat[0] = G;

    // t in {0,1}^3
    vector<MPoint> t = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};
    
    for (int i = 1; i <= iMax; i++) {
        GHat[i] = CalculatePyramid(GHat[i - 1], t, [](bool a, bool b) {return a || b;});
    }
}

int BoundingProxy::World2Voxel(double w, double min, double max, int res) {
    int result = (int) ((w - min) / (max - min) * (res - 1));
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
void BoundingProxy::VoxelizationCPU(int res) {
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

    double scale = res * res * 5.0;
    minX *= scale;
    minY *= scale;
    minZ *= scale;
    maxX *= scale;
    maxY *= scale;
    maxZ *= scale;

    for (unsigned int i = 0; i < vertices.length(); i++) {
        vertices[i] *= scale;
    }

    // Iterate through all triangles
    for (unsigned int i = 0; i < triangleIndices.length(); i += 3) {
        MPoint v0 = vertices[triangleIndices[i]];
        MPoint v1 = vertices[triangleIndices[i + 1]];
        MPoint v2 = vertices[triangleIndices[i + 2]];

        // Convert to voxel space
        int y0 = World2Voxel(v0.y, minY, maxY, res);
        int z0 = World2Voxel(v0.z, minZ, maxZ, res);

        int y1 = World2Voxel(v1.y, minY, maxY, res);
        int z1 = World2Voxel(v1.z, minZ, maxZ, res);

        int y2 = World2Voxel(v2.y, minY, maxY, res);
        int z2 = World2Voxel(v2.z, minZ, maxZ, res);

        // Compute bounding box in voxel space
        int minY_vox = std::min({ y0, y1, y2 });
        int maxY_vox = std::max({ y0, y1, y2 });
        int minZ_vox = std::min({ z0, z1, z2 });
        int maxZ_vox = std::max({ z0, z1, z2 });

        // Iterate over yz voxel columns
        for (int y = minY_vox; y <= maxY_vox; y++) {
            for (int z = minZ_vox; z <= maxZ_vox; z++) {
                double vy = Voxel2World(y, minY, maxY, res);
                double vz = Voxel2World(z, minZ, maxZ, res);
                vy += deltaP.y / 2.0;
                vz += deltaP.z / 2.0;

                // Check if voxel column is inside the triangle projection
                if (!InsideTriangleYZ(v0, v1, v2, vy, vz)) continue;

                // Compute intersection with triangle plane
                double xIntersect = IntersectTriangleX(v0, v1, v2, vy, vz);
                if (xIntersect == std::numeric_limits<double>::max()) {
                    continue;
                }
                int xIntersectVox = World2Voxel(xIntersect, minX, maxX, res);

                // Flip all voxels beyond the intersection
                for (int x = xIntersectVox; x < res; x++) {
                    G[x][y][z] = !G[x][y][z];
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
    if (fabs(n_ei_yz.x) < EPSILON) return std::numeric_limits<double>::max();

    return (-d_ei_yz - n_ei_yz.y * y - n_ei_yz.z * z) / n_ei_yz.x;
}

MStatus BoundingProxy::SelectMesh() {
    MSelectionList selection;
    MStatus status = selection.add(meshName);

    status = selection.getDagPath(0, meshPath);

    if (!meshPath.hasFn(MFn::kMesh)) {
        MGlobal::displayWarning("Selected object is not a mesh!");
        return MS::kFailure;
    }

    MDagPath transformPath = meshPath;
    transformPath.pop();

    MFnTransform transformFn(transformPath);
    meshName = transformFn.name();

    MString freezeCommand = "makeIdentity -apply true -t 1 -r 1 -s 1 -n 0 -pn 1 " + meshName + ";";
    MGlobal::executeCommand(freezeCommand);

    delete meshFn;
    meshFn = new MFnMesh(meshPath);
    return MS::kSuccess;
}

void BoundingProxy::ShowVoxel(vector<vector<vector<bool>>> grid, MString name) {
    int res = (int) grid.size();
    MString deleteGroupCmd =
        "if (`objExists " + name + "`) { "
        "delete " + name + "; "
        "} "
        "group -empty -name " + name + ";";
    MGlobal::executeCommand(deleteGroupCmd);

    for (int x = 0; x < res; x++) {
        for (int y = 0; y < res; y++) {
            for (int z = 0; z < res; z++) {
                if (grid[x][y][z]) {
                    // Compute world space position of voxel
                    double px = x * deltaP.x;
                    double py = y * deltaP.y;
                    double pz = z * deltaP.z;

                    // Create a cube for the voxel
                    string cubeName = "voxelCube_" + to_string(voxelCount);
                    string cubeCmd = "polyCube -w " + to_string(deltaP.x) +
                        " -h " + to_string(deltaP.y) +
                        " -d " + to_string(deltaP.z) +
                        " -n " + cubeName + ";";
                    MGlobal::executeCommand(MString(cubeCmd.c_str()));

                    // Move the cube to the voxel position
                    string moveCmd = "move " + to_string(px) + " " +
                        to_string(py) + " " +
                        to_string(pz) + " " + cubeName + ";";
                    MGlobal::executeCommand(MString(moveCmd.c_str()));

                    // Parent to group
                    string parentCmd = "parent " + cubeName + " " + name.asChar() + ";";
                    MGlobal::executeCommand(MString(parentCmd.c_str()));

                    voxelCount++;
                }
            }
        }
    }
}