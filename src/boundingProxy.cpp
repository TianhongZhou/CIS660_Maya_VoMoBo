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
        resetScaleField();
    } 
    else if (command == "generate") {
        int resolution = args.asInt(1);
        MString SE = args.asString(3);
        double baseScale = args.asDouble(4);
        meshName = args.asString(5);

        selectMesh();

        if (args.asString(2) == "cpu") {
            // Compute G
            voxelizationCPU(resolution);
            // Compute GHat
            pyramidGCPU();
            // Compute D
            dilationCPU(SE, baseScale);
            // Compute Dc
            connectedContourCPU();
            // Compute DcHat
            scaleAugmentedPyramidCPU();
            // Compute E
            erosionCPU();
            // Store E to outputG
            outputG = E;
            // Compute Mesh C
            cubeMarching();
            // CQEM on C
            simplifyMesh();
            // Show Mesh C
            createMayaMesh("final");
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

        selectMesh();

        if (args.asString(2) == "cpu") {
            voxelizationCPU(resolution);
            pyramidGCPU();
            dilationCPU(SE, baseScale);
            connectedContourCPU();
            scaleAugmentedPyramidCPU();
            erosionCPU();
        }
        else {
            // TODO: GPU
        }
        // Can show G, GHat[i], D, Dc, DcHat[i].first, E
        showVoxel(G, "G");
    }
    else {
        MGlobal::displayWarning("Unknown command argument!");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

// Deriving Error Quadrics based-on "Surface Simplification Using Quadric Error Metrics" (Garland, Heckbert)
void BoundingProxy::computeQuadricMatrices() {
    quadrics.resize(V.rows());
    for (int i = 0; i < F.rows(); i++) {
        Eigen::Vector3d v0 = V.row(F(i, 0));
        Eigen::Vector3d v1 = V.row(F(i, 1));
        Eigen::Vector3d v2 = V.row(F(i, 2));

        Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();
        double d = -normal.dot(v0);
        // p = [a b c d] represents the plane defined by the equation ax + by + cz + d = 0
        Eigen::Vector4d plane(normal.x(), normal.y(), normal.z(), d);

        quadrics[F(i, 0)].addPlane(plane);
        quadrics[F(i, 1)].addPlane(plane);
        quadrics[F(i, 2)].addPlane(plane);
    }
}

// CQEM based-on "Bounding Proxies for Shape Approximation Supplemental Materials"
double BoundingProxy::computeCollapseCost(Quadric& Q0, Quadric& Q1, Eigen::Vector3d& v_opt, int v0, int v1) {
    //// CQEM
    //vector<Triangle> triangles;
    //for (int i = 0; i < F.rows(); i++) {
    //    int v0_idx = F(i, 0);
    //    int v1_idx = F(i, 1);
    //    int v2_idx = F(i, 2);

    //    // 只考虑包含 v0 或 v1 的三角形
    //    if (v0_idx == v0 || v1_idx == v0 || v2_idx == v0 ||
    //        v0_idx == v1 || v1_idx == v1 || v2_idx == v1) {

    //        Eigen::Vector3d v0_pos = V.row(v0_idx).transpose();
    //        Eigen::Vector3d v1_pos = V.row(v1_idx).transpose();
    //        Eigen::Vector3d v2_pos = V.row(v2_idx).transpose();

    //        Eigen::Vector3d normal = (v1_pos - v0_pos).cross(v2_pos - v0_pos);
    //        normal.normalize();

    //        double d = -normal.dot(v0_pos);

    //        triangles.push_back(Triangle{v0_pos, v1_pos, v2_pos, normal, d});
    //    }
    //}

    //// 使用完整的 4x4 Quadric 矩阵
    //Eigen::Matrix4d Q4x4 = Q0.Q + Q1.Q;

    //// 检查 Q 是否是正定的
    //if (Q4x4.fullPivLu().rank() < 4) {
    //    return std::numeric_limits<double>::infinity();
    //}

    //// QP 变量（4维）
    //int num_constraints = (int) triangles.size();
    //Eigen::MatrixXd A(num_constraints, 4);
    //Eigen::VectorXd b(num_constraints);

    //// 构造 A 和 b 矩阵
    //for (int i = 0; i < num_constraints; i++) {
    //    A.row(i).head<3>() = triangles[i].normal.transpose();
    //    A(i, 3) = triangles[i].d;
    //    b(i) = 0;
    //}

    //// qpOASES 变量
    //qpOASES::SQProblem qp(4, num_constraints);  // 4 维变量（齐次坐标），num_constraints 个不等式约束
    //qpOASES::Options options;
    //options.setToMPC();
    //options.printLevel = qpOASES::PL_LOW;
    //qp.setOptions(options);

    //// 目标函数 H (对称矩阵) 和 g
    //qpOASES::real_t H[16], g[4] = {0, 0, 0, 0};  // g 是零向量
    //for (int i = 0; i < 4; i++) {
    //    for (int j = 0; j < 4; j++) {
    //        H[i * 4 + j] = Q4x4(i, j);
    //    }
    //}

    //// 线性不等式约束
    //vector<qpOASES::real_t> A_qp(num_constraints * 4);
    //vector<qpOASES::real_t> lb(num_constraints);
    //vector<qpOASES::real_t> ub(num_constraints);
    //qpOASES::real_t* A_qp_ptr = A_qp.data();
    //qpOASES::real_t* lb_ptr = lb.data();
    //qpOASES::real_t* ub_ptr = ub.data();
    //for (int i = 0; i < num_constraints; i++) {
    //    for (int j = 0; j < 4; j++) {
    //        A_qp[i * 4 + j] = A(i, j);
    //    }
    //    lb[i] = 0;
    //    ub[i] = qpOASES::INFTY;
    //}

    //// 初始化求解
    //int nWSR = 100;
    //qp.init(H, g, A_qp_ptr, nullptr, nullptr, lb_ptr, ub_ptr, nWSR);

    //// 结果
    //qpOASES::real_t v_opt_qp[4];
    //qp.getPrimalSolution(v_opt_qp);
    //v_opt = Eigen::Vector3d(v_opt_qp[0], v_opt_qp[1], v_opt_qp[2]);  // 去掉齐次坐标

    //// 计算最终的 cost
    //Eigen::Vector4d v_opt_homogeneous(v_opt[0], v_opt[1], v_opt[2], 1);
    //return v_opt_homogeneous.transpose() * Q4x4 * v_opt_homogeneous;
    
    // QEM
    // 使用完整的 4x4 Quadric 矩阵
    Eigen::Matrix4d Q4x4 = Q0.Q + Q1.Q;

    // 直接用 QEM 方式求解最优顶点 v_opt
    Eigen::Matrix3d A = Q4x4.block<3, 3>(0, 0);
    Eigen::Vector3d b = -Q4x4.block<3, 1>(0, 3);

    // 检查 A 是否可逆
    if (A.fullPivLu().rank() < 3) {
        // v_opt = (V.row(v0) + V.row(v1)) / 2; // 退化情况，使用中点
        return numeric_limits<double>::infinity();
    }

    v_opt = A.ldlt().solve(b);

    // 计算 QEM 误差 cost
    Eigen::Vector4d v_opt_homogeneous(v_opt[0], v_opt[1], v_opt[2], 1);
    double cost = v_opt_homogeneous.transpose() * Q4x4 * v_opt_homogeneous;

    return cost;
}

// Prevent collapse of an edge e = (v0, v1) for which ||v0 - v1|| > 4 * min(S[v0], S[v1])
bool BoundingProxy::edgeLengthExceedsThreshold(int v0, int v1) {
    Eigen::Vector3d p0 = V.row(v0);
    Eigen::Vector3d p1 = V.row(v1);

    double minX = V.col(0).minCoeff();
    double maxX = V.col(0).maxCoeff();
    double minY = V.col(1).minCoeff();
    double maxY = V.col(1).maxCoeff();
    double minZ = V.col(2).minCoeff();
    double maxZ = V.col(2).maxCoeff();

    int res = (int) S.size();
    int v0x = world2Voxel(p0.x(), minX, maxX, res);
    int v0y = world2Voxel(p0.y(), minY, maxY, res);
    int v0z = world2Voxel(p0.z(), minZ, maxZ, res);
    int v1x = world2Voxel(p1.x(), minX, maxX, res);
    int v1y = world2Voxel(p1.y(), minY, maxY, res);
    int v1z = world2Voxel(p1.z(), minZ, maxZ, res);

    double scale_v0 = S[v0x][v0y][v0z];
    double scale_v1 = S[v1x][v1y][v1z];

    return pow(v0x - v1x, 2.0) + pow(v0y - v1y, 2.0) + pow(v0z - v1z, 2.0) > 4 * std::min(scale_v0, scale_v1);
}

void BoundingProxy::performCollapse(EdgeCollapse& ec) {
    int v0 = ec.v0;
    int v1 = ec.v1;
    Eigen::Vector3d v_opt = ec.v_opt;

    V.row(v0) = v_opt;

    quadrics[v0].Q += quadrics[v1].Q;

    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F(i, j) == v1) {
                F(i, j) = v0;
            }
        }
    }

    vector<Eigen::Vector3i> newFaces;
    for (int i = 0; i < F.rows(); i++) {
        if (F(i, 0) != F(i, 1) && F(i, 1) != F(i, 2) && F(i, 0) != F(i, 2)) {
            newFaces.push_back(F.row(i));
        }
    }

    F.resize(newFaces.size(), 3);
    for (int i = 0; i < newFaces.size(); i++) {
        F.row(i) = newFaces[i];
    }

    priority_queue<EdgeCollapse> newQueue;
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            int a = F(i, j);
            int b = F(i, (j + 1) % 3);
            if (a > b) {
                continue;
            }

            Eigen::Vector3d v_new_opt;
            double cost = computeCollapseCost(quadrics[a], quadrics[b], v_new_opt, a, b);

            if (cost < numeric_limits<double>::infinity()) {
                newQueue.push({a, b, cost, v_new_opt});
            }
        }
    }
    collapseQueue.swap(newQueue);
}

void BoundingProxy::simplifyMesh() {
    quadrics.clear();
    computeQuadricMatrices();

    priority_queue<EdgeCollapse>().swap(collapseQueue);

    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            int v0 = F(i, j);
            int v1 = F(i, (j + 1) % 3);
            if (v0 > v1) {
                continue;
            }

            Eigen::Vector3d v_opt;
            double cost = computeCollapseCost(quadrics[v0], quadrics[v1], v_opt, v0, v1);

            
            if (cost < numeric_limits<double>::infinity()) {
                collapseQueue.push({v0, v1, cost, v_opt});
            }
        }
    }

    while (!collapseQueue.empty()) {
        EdgeCollapse ec = collapseQueue.top();
        collapseQueue.pop();

        if (edgeLengthExceedsThreshold(ec.v0, ec.v1)) {
            continue;
        }

        performCollapse(ec);
    }
}

// Convert Eigen V and F to Maya mesh
void BoundingProxy::createMayaMesh(MString name) {
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
void BoundingProxy::cubeMarching() {
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

    igl::copyleft::marching_cubes(Sm, GV, newN, newN, newN, 0, V, F);
}

// Algorithm 2 - Parallel spatially varying erosion
void BoundingProxy::erosionCPU() {
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
void BoundingProxy::scaleAugmentedPyramidCPU() {
    int N0 = (int) Dc.size();
    int iMax = (int) log2(N0);
    // Dc0[p] = (Dc[p], S[p]) 
    DcHat[0].first = Dc;
    DcHat[0].second = S;

    // t in {0,1}^3
    vector<MPoint> t = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};

    for (int i = 1; i <= iMax; i++) {
        DcHat[i].first = calculatePyramid(DcHat[i - 1].first, t, [](bool a, bool b) {return a || b;});
        DcHat[i].second = calculatePyramid(DcHat[i - 1].second, t, [](double a, double b) {return max(a, b);});
    }
}

// Compute Dc from D using 6-connected contour
void BoundingProxy::connectedContourCPU() {
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
void BoundingProxy::dilationCPU(MString SE, double baseScale) {
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

void BoundingProxy::resetScaleField() {
    editedS = false;
    S.clear();
}

// for all p in Ni^3, Gi[p] = max_t G^{i - 1}[2p + t];
// for all p in Ni^3, Dci[p] = max_t Dc^{i - 1}[2p + t];
template <typename T, typename CombineOp>
vector<vector<vector<T>>> BoundingProxy::calculatePyramid(vector<vector<vector<T>>> prev, vector<MPoint> t, CombineOp combine) {
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
void BoundingProxy::pyramidGCPU() {
    int N0 = (int) G.size();
    int iMax = (int) log2(N0);
    // G0[p] = G[p] 
    GHat[0] = G;

    // t in {0,1}^3
    vector<MPoint> t = {{0,0,0}, {0,0,1}, {0,1,0}, {1,0,0},
                        {0,1,1}, {1,0,1}, {1,1,0}, {1,1,1}};
    
    for (int i = 1; i <= iMax; i++) {
        GHat[i] = calculatePyramid(GHat[i - 1], t, [](bool a, bool b) {return a || b;});
    }
}

int BoundingProxy::world2Voxel(double w, double min, double max, int res) {
    int result = (int) ((w - min) / (max - min) * (res - 1));
    result = std::max(result, 0);
    result = std::min(result, res - 1);
    return result;
}

double BoundingProxy::voxel2World(int v, double min, double max, int res) {
    return min + (v / (double) (res - 1)) * (max - min);
}

MVector BoundingProxy::crossProduct(MVector a, MVector b) {
    return MVector(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// Method based-on "Fast Parallel Surface and Solid Voxelization on GPUs" (Schwarz, Seidel)
void BoundingProxy::voxelizationCPU(int res) {
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
        int y0 = world2Voxel(v0.y, minY, maxY, res);
        int z0 = world2Voxel(v0.z, minZ, maxZ, res);

        int y1 = world2Voxel(v1.y, minY, maxY, res);
        int z1 = world2Voxel(v1.z, minZ, maxZ, res);

        int y2 = world2Voxel(v2.y, minY, maxY, res);
        int z2 = world2Voxel(v2.z, minZ, maxZ, res);

        // Compute bounding box in voxel space
        int minY_vox = std::min({ y0, y1, y2 });
        int maxY_vox = std::max({ y0, y1, y2 });
        int minZ_vox = std::min({ z0, z1, z2 });
        int maxZ_vox = std::max({ z0, z1, z2 });

        // Iterate over yz voxel columns
        for (int y = minY_vox; y <= maxY_vox; y++) {
            for (int z = minZ_vox; z <= maxZ_vox; z++) {
                double vy = voxel2World(y, minY, maxY, res);
                double vz = voxel2World(z, minZ, maxZ, res);
                vy += deltaP.y / 2.0;
                vz += deltaP.z / 2.0;

                // Check if voxel column is inside the triangle projection
                if (!insideTriangleYZ(v0, v1, v2, vy, vz)) continue;

                // Compute intersection with triangle plane
                double xIntersect = intersectTriangleX(v0, v1, v2, vy, vz);
                if (xIntersect == std::numeric_limits<double>::max()) {
                    continue;
                }
                int xIntersectVox = world2Voxel(xIntersect, minX, maxX, res);

                // Flip all voxels beyond the intersection
                for (int x = xIntersectVox; x < res; x++) {
                    G[x][y][z] = !G[x][y][z];
                }
            }
        }
    }
}

double BoundingProxy::edgeFunction(MPoint a, MPoint b, MVector p) {
    return (p.x - a.y) * (b.z - a.z) - (p.y - a.z) * (b.y - a.y);
};

bool BoundingProxy::insideTriangleYZ(MPoint v0, MPoint v1, MPoint v2, double y, double z) {
    MVector p(y, z);

    // Compute signed area tests (edge function tests)
    double w0 = edgeFunction(v0, v1, p);
    double w1 = edgeFunction(v1, v2, p);
    double w2 = edgeFunction(v2, v0, p);

    return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
}

double BoundingProxy::intersectTriangleX(MPoint v0, MPoint v1, MPoint v2, double y, double z) {
    // Triangle plane equation: Ax + By + Cz + D = 0
    MVector n_ei_yz = crossProduct(v1 - v0, v2 - v0);
    double d_ei_yz = -n_ei_yz * v0;

    // Solve for x: x = (-D - By - Cz) / A
    if (fabs(n_ei_yz.x) < EPSILON) return std::numeric_limits<double>::max();

    return (-d_ei_yz - n_ei_yz.y * y - n_ei_yz.z * z) / n_ei_yz.x;
}

MStatus BoundingProxy::selectMesh() {
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

void BoundingProxy::showVoxel(vector<vector<vector<bool>>> grid, MString name) {
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