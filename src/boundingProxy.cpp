#include "../Header.h"
#include "boundingProxy.h"

std::vector<std::vector<std::vector<double>>> BoundingProxy::S;
std::vector<std::vector<std::vector<double>>> BoundingProxy::prevS;
bool BoundingProxy::editedS = false;
MPoint BoundingProxy::lastEditP = MPoint(DBL_MAX, DBL_MAX, DBL_MAX);

BoundingProxy::BoundingProxy() 
    : meshFn(nullptr), voxelCount(0)
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

        int resolution = args.asInt(1);
        double baseScale = args.asDouble(2);
        meshName = args.asString(3);

        selectMesh();

        showScaleFieldColors(resolution, baseScale);
    } 
    else if (command == "brush_size") {
        MString mode = args.asString(1);
        double baseScale = args.asDouble(2);
        meshName = args.asString(3);
        double radius = args.asDouble(4);
        double targetScale = args.asDouble(5);

        selectMesh();

        if (lastEditP.x != DBL_MAX) {
            S = prevS;
            editScaleField(lastEditP.x, lastEditP.y, lastEditP.z, radius, targetScale, mode, baseScale, true);
        }
    }
    else if (command == "edit_scale_field") {
        MPoint rayOrigin(args.asDouble(1), args.asDouble(2), args.asDouble(3));
        MVector rayDir(args.asDouble(4), args.asDouble(5), args.asDouble(6));
        meshName = args.asString(7);
        double radius = args.asDouble(8);
        double targetScale = args.asDouble(9);
        MString mode = args.asString(10);
        double baseScale = args.asDouble(11);

        selectMesh();

        MPoint intersectionPoint;
        if (raycastToMesh(rayOrigin, rayDir, intersectionPoint)) {
            editScaleField(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, 0.0, targetScale, mode, baseScale, false);
            prevS = S;
            editScaleField(intersectionPoint.x, intersectionPoint.y, intersectionPoint.z, radius, targetScale, mode, baseScale, true);
            lastEditP = intersectionPoint;
        }
    }
    else if (command == "show_scale_field") {
        int resolution = args.asInt(1);
        double baseScale = args.asDouble(2);
        meshName = args.asString(3);

        selectMesh();

        showScaleFieldColors(resolution, baseScale);
    }
    else if (command == "generate") {
        int resolution = args.asInt(1);
        MString SE = args.asString(3);
        double baseScale = args.asDouble(4);
        meshName = args.asString(5);
        double maxError = args.asDouble(6);
        MString simplifyMethod = args.asString(7);

        selectMesh();

        if (args.asString(2) == "cpu") {
            
            


            // Compute G
            voxelizationCPU(resolution);
            // Compute GHat
            pyramidGCPU();
            // Compute D
            dilationCPU(SE, baseScale);
            // Compute Dc
            connectedContourCPU(D);
            // Compute DcHat
            scaleAugmentedPyramidCPU();
            // Compute E
            erosionCPU();
            // Store E to outputG
            outputG = E;
            // Compute Mesh C
            cubeMarching();
            // CQEM on C
            simplifyMesh(maxError, simplifyMethod);
            // Show Mesh C
            createMayaMesh("final");
        }
        else {

            resetScaleField();
            S.assign(resolution,
                vector<vector<double>>(resolution,
                    vector<double>(resolution, baseScale)));
            //editedS = true;

            // 1) Rasterize into flatI via your existing triangle kernel
            MPointArray verts;
            meshFn->getPoints(verts, MSpace::kWorld);
            MIntArray triCounts, triIdx;
            meshFn->getTriangles(triCounts, triIdx);
            int numTris = triIdx.length() / 3;

            vector<GpuTriangle> tris(numTris);
            for (int t = 0; t < numTris; ++t) {
                int i0 = triIdx[3 * t + 0], i1 = triIdx[3 * t + 1], i2 = triIdx[3 * t + 2];
                auto& A = verts[i0], & B = verts[i1], & C = verts[i2];
                tris[t].v0x = A.x; tris[t].v0y = A.y; tris[t].v0z = A.z;
                tris[t].v1x = B.x; tris[t].v1y = B.y; tris[t].v1z = B.z;
                tris[t].v2x = C.x; tris[t].v2y = C.y; tris[t].v2z = C.z;
            }

            auto bb = meshFn->boundingBox();
            double scale = double(resolution) * resolution * 5.0;
            double minX = bb.min().x * scale, maxX = bb.max().x * scale;
            double minY = bb.min().y * scale, maxY = bb.max().y * scale;
            double minZ = bb.min().z * scale, maxZ = bb.max().z * scale;
            for (auto& T : tris) {
                T.v0x *= scale; T.v0y *= scale; T.v0z *= scale;
                T.v1x *= scale; T.v1y *= scale; T.v1z *= scale;
                T.v2x *= scale; T.v2y *= scale; T.v2z *= scale;
            }

            size_t N = size_t(resolution) * resolution * resolution;
            vector<unsigned int> flatI(N);
            rasterizeMeshOnGpuTriangleAoS(
                tris.data(), numTris, resolution,
                minX, maxX, minY, maxY, minZ, maxZ,
                flatI.data()
            );

            // 2) Build flatG0 + 3D G
            vector<unsigned char> flatG0(N);
            G.assign(resolution,
                vector<vector<bool>>(resolution,
                    vector<bool>(resolution, false)));
            for (size_t i = 0; i < N; ++i) {
                bool occ = (flatI[i] & 1u) != 0;
                flatG0[i] = occ ? 1 : 0;
                int x = i / (resolution * resolution);
                int y = (i / resolution) % resolution;
                int z = i % resolution;
                G[x][y][z] = occ;
            }

            // 3) recompute deltaP in world units
            MPoint bbMin = bb.min(), bbMax = bb.max();
            deltaP = MPoint(
                (bbMax.x - bbMin.x) / resolution,
                (bbMax.y - bbMin.y) / resolution,
                (bbMax.z - bbMin.z) / resolution
            );

            // 4) build Boolean pyramid levels GH[0..iMax] on GPU
            int iMax = int(std::log2(double(resolution)));
            vector<vector<unsigned char>> GH(iMax + 1);
            GH[0] = std::move(flatG0);
            for (int lvl = 1, dim = resolution; lvl <= iMax; ++lvl) {
                int prevDim = dim;  dim >>= 1;
                GH[lvl].resize(size_t(dim) * dim * dim);
                computePyramidLevelOnGpu(
                    GH[lvl - 1].data(),
                    GH[lvl].data(),
                    prevDim
                );
            }

            // 5) pack levels 1..iMax into one buffer
            size_t ghTotal = 0;
            for (int lvl = 1; lvl <= iMax; ++lvl) {
                int d = resolution >> lvl;
                ghTotal += size_t(d) * d * d;
            }
            vector<unsigned char> GH_packed;
            GH_packed.reserve(ghTotal);
            for (int lvl = 1; lvl <= iMax; ++lvl) {
                GH_packed.insert(
                    GH_packed.end(),
                    GH[lvl].begin(), GH[lvl].end()
                );
            }

            // 6) full spatially‐varying dilation on GPU
            vector<double> flatS(N, baseScale);      
            vector<unsigned char> flatD(N);
            spatiallyVaryingDilationOnGpu(
                GH[0].data(),
                GH_packed.data(),
                flatS.data(),
                flatD.data(),
                resolution,
                iMax,
                deltaP.x,
                (SE == "cube") ? 1 : 0
            );

            // 7) unpack flatD → D 3D
            D.assign(resolution,
                vector<vector<bool>>(resolution,
                    vector<bool>(resolution, false)));
            for (int x = 0; x < resolution; ++x)
                for (int y = 0; y < resolution; ++y)
                    for (int z = 0; z < resolution; ++z) {
                        size_t idx = (x * resolution + y) * resolution + z;
                        D[x][y][z] = (flatD[idx] != 0);
                    }

            // 8) CPU‐side *connected contour* → Dc
            connectedContourCPU(D);
            // 9) CPU‐side *scale‐augmented* pyramid → DcHat
            scaleAugmentedPyramidCPU();
            // 10) CPU erosion on DcHat → E
            erosionCPU();
            outputG = E;         

            // 11) March & simplify exactly as in your CPU path
            cubeMarching();
            simplifyMesh(maxError, simplifyMethod);
            createMayaMesh("final");



            //connectedContourCPU(D);
            //scaleAugmentedPyramidCPU();
            ////erosionCPU();               // now safe: G, DcHat, S all correct
            //outputG = E;
            //cubeMarching();
            //simplifyMesh(maxError, simplifyMethod);
            //createMayaMesh("final");

        }
    }
    else if (command == "show_voxel") {
        int resolution = args.asInt(1);
        MString SE = args.asString(3);
        double baseScale = args.asDouble(4);
        meshName = args.asString(5);

        selectMesh();

        if (args.asString(2) == "cpu") {
            voxelizationCPU(resolution);
            connectedContourCPU(G);
        }
        else {

            

            MPointArray verts;
            meshFn->getPoints(verts, MSpace::kWorld);
            MIntArray triCounts, triIdx;
            meshFn->getTriangles(triCounts, triIdx);
            int numTris = triIdx.length() / 3;

            vector<GpuTriangle> tris(numTris);
            for (int t = 0; t < numTris; ++t) {
                int i0 = triIdx[3 * t], i1 = triIdx[3 * t + 1], i2 = triIdx[3 * t + 2];
                auto& A = verts[i0], & B = verts[i1], & C = verts[i2];
                tris[t].v0x = A.x; tris[t].v0y = A.y; tris[t].v0z = A.z;
                tris[t].v1x = B.x; tris[t].v1y = B.y; tris[t].v1z = B.z;
                tris[t].v2x = C.x; tris[t].v2y = C.y; tris[t].v2z = C.z;
            }

            auto bb = meshFn->boundingBox();
            double scale = double(resolution) * resolution * 5.0;
            double minX = bb.min().x * scale, maxX = bb.max().x * scale;
            double minY = bb.min().y * scale, maxY = bb.max().y * scale;
            double minZ = bb.min().z * scale, maxZ = bb.max().z * scale;
            for (int t = 0; t < numTris; ++t) {
                tris[t].v0x *= scale; tris[t].v0y *= scale; tris[t].v0z *= scale;
                tris[t].v1x *= scale; tris[t].v1y *= scale; tris[t].v1z *= scale;
                tris[t].v2x *= scale; tris[t].v2y *= scale; tris[t].v2z *= scale;
            }

            size_t N = size_t(resolution) * resolution * resolution;
            vector<unsigned int> flatI(N);
            rasterizeMeshOnGpuTriangleAoS(
                tris.data(), numTris, resolution,
                minX, maxX, minY, maxY, minZ, maxZ,
                flatI.data()
            );

            G.assign(resolution,
                vector<vector<bool>>(resolution, vector<bool>(resolution)));
            for (int x = 0; x < resolution; ++x)
                for (int y = 0; y < resolution; ++y)
                    for (int z = 0; z < resolution; ++z) {
                        size_t idx = (x * resolution + y) * resolution + z;
                        G[x][y][z] = (flatI[idx] & 1u) != 0;
                    }

            MPoint bbMin = meshFn->boundingBox().min();
            MPoint bbMax = meshFn->boundingBox().max();
            deltaP = MPoint(
                (bbMax.x - bbMin.x) / resolution,
                (bbMax.y - bbMin.y) / resolution,
                (bbMax.z - bbMin.z) / resolution
            );

            connectedContourCPU(G);
        }
       
        // Can show G, GHat[i], D, Dc, DcHat[i].first, E
        showVoxel(Dc, "voxels");
    }
    else {
        MGlobal::displayWarning("Unknown command argument!");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

double BoundingProxy::gamma(double t, double s, double sigma) {
    if (t >= 0.0 && t < s) {
        return 1.0;
    }
    else if (t >= s && t < s + sigma) {
        return pow(1.0 - pow((t - s) / sigma, 2.0), 2.0);
    }
    else {
        return 0.0;
    }
}

double BoundingProxy::phi(MPoint x, double sprev, double s, double sigma, MPoint p) {
    double norm = x.distanceTo(p);
    double g = gamma(norm, s, sigma);
    return (1.0 - g) * sprev + g * s;
}

void BoundingProxy::editScaleField(double wx, double wy, double wz, double sigma, double s, MString mode, double bs, bool show) {
    editedS = true;

    int vx = (int)world2Voxel(wx, bboxMin.x, bboxMax.x, (int)S.size());
    int vy = (int)world2Voxel(wy, bboxMin.y, bboxMax.y, (int)S.size());
    int vz = (int)world2Voxel(wz, bboxMin.z, bboxMax.z, (int)S.size());

    for (int x = max(0, vx - (int)sigma); x < min((int)S.size(), vx + (int)sigma); x++) {
        for (int y = max(0, vy - (int)sigma); y < min((int)S.size(), vy + (int)sigma); y++) {
            for (int z = max(0, vz - (int)sigma); z < min((int)S.size(), vz + (int)sigma); z++) {
                MPoint X(x, y, z);
                MPoint P(vx, vy, vz);
                double Sx = S[x][y][z];
                if (mode == "increase") {
                    S[x][y][z] = max(phi(X, Sx, s, sigma, P), Sx);
                }
                else if (mode == "decrease") {
                    S[x][y][z] = min(phi(X, Sx, s, sigma, P), Sx);
                }
                else {
                    S[x][y][z] = phi(X, Sx, bs, sigma, P);
                }
            }
        }
    }

    if (show) {
        showScaleFieldColors((int)S.size(), bs);
    }
}

bool BoundingProxy::raycastToMesh(MPoint origin, MVector dir, MPoint& outHitPoint) {
    MFloatPoint floatHit;
    float rayParam, bary1, bary2;
    int faceIdx, triIdx;

    MStatus stat;

    bool hit = meshFn->closestIntersection(
        MFloatPoint(origin),
        MFloatVector(dir),
        nullptr,            // all faces
        nullptr,            // all triangles
        false,              // idsSorted
        MSpace::kWorld,
        1e10f,              // max ray distance
        false,              // test both directions
        nullptr,            // no accel struct
        floatHit,           // output: intersection point
        &rayParam,          // output: ray t
        &faceIdx,           // output: face idx
        &triIdx,            // output: triangle idx
        &bary1,             // output: bary coord 1
        &bary2,             // output: bary coord 2
        1e-6f,              // tolerance
        &stat               // output: status
    );

    if (hit) {
        outHitPoint = MPoint(floatHit);
    }

    return hit;
}

void BoundingProxy::showScaleFieldColors(int res, double baseScale) {
    if (!editedS) {
        // Initialize S if haven't
        S = vector<vector<vector<double>>>(res, vector<vector<double>>(res, vector<double>(res, baseScale)));
    }

    MPointArray vertices;
    meshFn->getPoints(vertices, MSpace::kWorld);

    MColorArray vertexColors;
    MIntArray vertexIndices;

    // Compute minS and maxS
    double minS = DBL_MAX, maxS = DBL_MIN;
    for (int x = 0; x < S.size(); x++) {
        for (int y = 0; y < S[x].size(); y++) {
            for (int z = 0; z < S[x][y].size(); z++) {
                double sVal = S[x][y][z];
                if (sVal < minS) minS = sVal;
                if (sVal > maxS) maxS = sVal;
            }
        }
    }

    for (unsigned int i = 0; i < vertices.length(); i++) {
        MPoint p = vertices[i];
        // Convert p to voxel space
        int vx = (int)world2Voxel(p.x, bboxMin.x, bboxMax.x, (int)S.size());
        int vy = (int)world2Voxel(p.y, bboxMin.y, bboxMax.y, (int)S.size());
        int vz = (int)world2Voxel(p.z, bboxMin.z, bboxMax.z, (int)S.size());

        double sVal = S[vx][vy][vz];
        double normS;
        if (fabs(maxS - minS) < 1e-8) {
            normS = 0.5;
        }
        else {
            normS = (sVal - minS) / (maxS - minS);
        }

        // Color mapping
        MColor color((float) normS, (float) normS, (float) normS);
        vertexColors.append(color);
        vertexIndices.append((int)i);
    }

    meshFn->setVertexColors(vertexColors, vertexIndices);
}

void BoundingProxy::simplifyMesh(double maxError, MString method) {
    Eigen::MatrixXd V_backup = V;
    Eigen::Vector3d minBound, maxBound;
    minBound << V.col(0).minCoeff(), V.col(1).minCoeff(), V.col(2).minCoeff();
    maxBound << V.col(0).maxCoeff(), V.col(1).maxCoeff(), V.col(2).maxCoeff();

    V.col(0) = (V.col(0).array() - minBound[0]) / (maxBound[0] - minBound[0]);
    V.col(1) = (V.col(1).array() - minBound[1]) / (maxBound[1] - minBound[1]);
    V.col(2) = (V.col(2).array() - minBound[2]) / (maxBound[2] - minBound[2]);

    MyMesh mesh;

    vector<MyMesh::VertexHandle> vHandles;
    for (int i = 0; i < V.rows(); i++) {
        vHandles.push_back(mesh.add_vertex(MyMesh::Point(V(i, 0), V(i, 1), V(i, 2))));
    }

    for (int i = 0; i < F.rows(); i++) {
        mesh.add_face(vHandles[F(i, 0)], vHandles[F(i, 1)], vHandles[F(i, 2)]);
    }

    Decimater decimater(mesh);

    HModQuadric modQuadric;
    decimater.add(modQuadric);

    decimater.module(modQuadric).set_max_err(maxError * 1e-5);

    decimater.initialize();
    double minX = V.col(0).minCoeff();
    double maxX = V.col(0).maxCoeff();
    double minY = V.col(1).minCoeff();
    double maxY = V.col(1).maxCoeff();
    double minZ = V.col(2).minCoeff();
    double maxZ = V.col(2).maxCoeff();
    decimater.decimate(minX, maxX, minY, maxY, minZ, maxZ, S, string(method.asChar()));

    mesh.garbage_collection();

    mesh.triangulate();

    V.resize(mesh.n_vertices(), 3);
    F.resize(mesh.n_faces(), 3);

    int index = 0;
    map<MyMesh::VertexHandle, int> vIndexMap;
    for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        MyMesh::Point p = mesh.point(*v_it);
        V.row(index) << p[0], p[1], p[2];
        vIndexMap[*v_it] = index++;
    }

    index = 0;
    for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it);
        F.row(index++) << vIndexMap[*fv_it], vIndexMap[*(++fv_it)], vIndexMap[*(++fv_it)];
    }

    V.col(0) = V.col(0).array() * (maxBound[0] - minBound[0]) + minBound[0];
    V.col(1) = V.col(1).array() * (maxBound[1] - minBound[1]) + minBound[1];
    V.col(2) = V.col(2).array() * (maxBound[2] - minBound[2]) + minBound[2];
}

// Convert Eigen V and F to Maya mesh
void BoundingProxy::createMayaMesh(MString name) {
    MSelectionList selList;
    if (MGlobal::getSelectionListByName(name, selList) == MS::kSuccess && !selList.isEmpty()) {
        MString deleteCmd = "delete " + name + ";";
        MGlobal::executeCommand(deleteCmd);
    }

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

    // Align mesh to original mesh
    MBoundingBox bboxB = meshFn.boundingBox();
    MPoint centerOffset = (bboxMin + bboxMax) / 2.0 - bboxB.center();
    MString moveCmd = "xform -ws -translation ";
    moveCmd += centerOffset.x; moveCmd += " ";
    moveCmd += centerOffset.y; moveCmd += " ";
    moveCmd += centerOffset.z; moveCmd += " ";
    moveCmd += name + ";";
    MGlobal::executeCommand(moveCmd);

    // Shading the mesh generated
    MString matName = "semiTransparentMat";
    MString shadingGroup = matName + "SG";
    MGlobal::executeCommand("shadingNode -asShader lambert -name " + matName);
    MGlobal::executeCommand("setAttr \"" + matName + ".transparency\" -type double3 0.8 0.8 0.8");
    MGlobal::executeCommand("setAttr \"" + matName + ".color\" -type double3 1 0 0");
    MGlobal::executeCommand("sets -renderable true -noSurfaceShader true -empty -name " + shadingGroup);
    MGlobal::executeCommand("connectAttr -f " + matName + ".outColor " + shadingGroup + ".surfaceShader");
    MGlobal::executeCommand("sets -e -forceElement " + shadingGroup + " " + name + ";");

    // Remeshing
    MBoundingBox bbox = meshFn.boundingBox();
    double maxDim = std::max({ bbox.width(), bbox.height(), bbox.depth() });
    double edgeLen = maxDim * 0.085;
    MString remeshCmd = "polyRemesh ";
    remeshCmd += "-maxEdgeLength ";
    remeshCmd += edgeLen;
    remeshCmd += "-collapseThreshold 1.0 ";
    remeshCmd += name + ";";
    MGlobal::executeCommand(remeshCmd);

    // Bind mesh if joints exits
    MDagPath shapePath = meshPath;
    shapePath.extendToShape();
    MObject shapeNode = shapePath.node();

    MItDependencyGraph dgIter(
        shapeNode,
        MFn::kSkinClusterFilter,
        MItDependencyGraph::kUpstream,
        MItDependencyGraph::kDepthFirst, 
        MItDependencyGraph::kPlugLevel 
    );

    if (!dgIter.isDone()) {
        MObject skinClusterObj = dgIter.currentItem();
        MFnSkinCluster skinFn(skinClusterObj);

        MDagPathArray jointPaths;
        skinFn.influenceObjects(jointPaths);

        for (unsigned int i = 0; i < jointPaths.length(); ++i) {
            MGlobal::executeCommand("select -add " + jointPaths[i].fullPathName());
        }

        MGlobal::executeCommand("select -add " + name);
        MGlobal::executeCommand("skinCluster -toSelectedBones -bindMethod 0 -skinMethod 0 -normalizeWeights 1;");
        MString cmd;
        cmd += "copySkinWeights -noMirror -surfaceAssociation closestPoint -influenceAssociation closestJoint ";
        cmd += "-normalize true ";
        cmd += meshName + " " + name + ";";
        MGlobal::executeCommand(cmd);
    }
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
                    // Average Neighborhood
                    double sum = 0.0;
                    int count = 0;
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dz = -1; dz <= 1; dz++) {
                                int nx = x - 1 + dx, ny = y - 1 + dy, nz = z - 1 + dz;
                                if (nx >= 0 && ny >= 0 && nz >= 0 && nx < N && ny < N && nz < N) {
                                    sum += outputG[nx][ny][nz] ? 1.0 : 0.0;
                                    count++;
                                }
                            }
                        }
                    }
                    Sm(index, 0) = sum / count;
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
void BoundingProxy::connectedContourCPU(vector<vector<vector<bool>>> grid) {
    int N = (int)grid.size();
    vector<MPoint> r = {{0,0,1}, {0,1,0}, {1,0,0}, {0,0,-1}, {0,-1,0}, {-1,0,0}};
    Dc = grid;

    for (int x = 1; x < N - 1; x++) {
        for (int y = 1; y < N - 1; y++) {
            for (int z = 1; z < N - 1; z++) {
                if (grid[x][y][z]) {
                    bool flag = true;
                    for (int i = 0; i < r.size(); i++) {
                        MPoint p = MPoint(x, y, z) + r[i];
                        if (!grid[(int) p.x][(int) p.y][(int) p.z]) {
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

double BoundingProxy::world2Voxel(double w, double min, double max, int res) {
    double result = (w - min) / (max - min) * (res - 1);
    result = std::max(result, 0.0);
    result = std::min(result, res - 1.0);
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

    // Get bounding box
    MPoint min = bboxMin;
    MPoint max = bboxMax;

    double minX = min.x, minY = min.y, minZ = min.z;
    double maxX = max.x, maxY = max.y, maxZ = max.z;
    deltaP = MPoint((maxX - minX) / res, (maxY - minY) / res, (maxZ - minZ) / res);

    double scale = res * res * 5.0;
    minX *= scale; minY *= scale; minZ *= scale;
    maxX *= scale; maxY *= scale; maxZ *= scale;

    for (unsigned int i = 0; i < vertices.length(); i++) {
        vertices[i] *= scale;
    }

    // Triangle processing function
    for (unsigned int i = 0; i < triangleIndices.length(); i += 3) {
        MPoint v0 = vertices[triangleIndices[i]];
        MPoint v1 = vertices[triangleIndices[i + 1]];
        MPoint v2 = vertices[triangleIndices[i + 2]];

        int y0 = (int)world2Voxel(v0.y, minY, maxY, res);
        int z0 = (int)world2Voxel(v0.z, minZ, maxZ, res);
        int y1 = (int)world2Voxel(v1.y, minY, maxY, res);
        int z1 = (int)world2Voxel(v1.z, minZ, maxZ, res);
        int y2 = (int)world2Voxel(v2.y, minY, maxY, res);
        int z2 = (int)world2Voxel(v2.z, minZ, maxZ, res);

        int minY_vox = std::min({ y0, y1, y2 });
        int maxY_vox = std::max({ y0, y1, y2 });
        int minZ_vox = std::min({ z0, z1, z2 });
        int maxZ_vox = std::max({ z0, z1, z2 });

        for (int y = minY_vox; y <= maxY_vox; y++) {
            for (int z = minZ_vox; z <= maxZ_vox; z++) {
                double vy = voxel2World(y, minY, maxY, res) + deltaP.y / 2.0;
                double vz = voxel2World(z, minZ, maxZ, res) + deltaP.z / 2.0;
                if (!insideTriangleYZ(v0, v1, v2, vy, vz)) continue;

                double xIntersect = intersectTriangleX(v0, v1, v2, vy, vz);
                if (xIntersect == numeric_limits<double>::max()) continue;
                int xIntersectVox = (int)world2Voxel(xIntersect, minX, maxX, res);

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
    MGlobal::executeCommand("select -cl;");
    MGlobal::executeCommand("select " + meshName + ";");

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

    delete meshFn;
    meshFn = new MFnMesh(meshPath);

    worldMatrix = meshPath.inclusiveMatrix();
    bboxMin = meshFn->boundingBox().min() * worldMatrix;
    bboxMax = meshFn->boundingBox().max() * worldMatrix;

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



