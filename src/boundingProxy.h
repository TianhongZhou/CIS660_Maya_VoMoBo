#ifndef BOUNDING_PROXY_H
#define BOUNDING_PROXY_H
#include <maya/MPxCommand.h>
#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MBoundingBox.h>
#include <maya/MString.h>
#include <maya/MFnTransform.h>
#include <maya/MFnSet.h>
#include <maya/MObject.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <stack>
#include <queue>
#include <igl/copyleft/marching_cubes.h>
#include <Eigen/Dense>
#include <qpOASES.hpp>
using namespace std;

constexpr double EPSILON = 1e-6;

struct Quadric {
    Eigen::Matrix4d Q;

    Quadric() {
        Q.setZero();
    }

    void addPlane(const Eigen::Vector4d& plane) {
        Q += plane * plane.transpose();
    }
};

struct EdgeCollapse {
    int v0, v1;
    double cost;
    Eigen::Vector3d v_opt;

    bool operator<(const EdgeCollapse& other) const {
        return cost > other.cost;
    }
};

struct Triangle {
    Eigen::Vector3d v0, v1, v2;
    Eigen::Vector3d normal; 
    double d;

    Triangle(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
        const Eigen::Vector3d& normal, double d)
        : v0(v0), v1(v1), v2(v2), normal(normal), d(d) {}
};

class BoundingProxy : public MPxCommand {
public:
    MDagPath meshPath;
    MFnMesh* meshFn;
    MString meshName;
    vector<vector<vector<bool>>> G;
    MVector deltaP;
    unordered_map<int, vector<vector<vector<bool>>>> GHat;
    vector<vector<vector<bool>>> D;
    vector<vector<vector<bool>>> Dc;
    vector<vector<vector<double>>> S;
    unordered_map<int, pair<vector<vector<vector<bool>>>, vector<vector<vector<double>>>>> DcHat;
    bool editedS;
    vector<vector<vector<bool>>> E;
    vector<vector<vector<bool>>> outputG;
    int voxelCount;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    vector<Quadric> quadrics;
    priority_queue<EdgeCollapse> collapseQueue;

    BoundingProxy();
    ~BoundingProxy();

    static void* creator();
    MStatus doIt(const MArgList&) override;
    MStatus selectMesh();
    void voxelizationCPU(int res);
    int world2Voxel(double w, double min, double max, int res);
    double voxel2World(int v, double min, double max, int res);
    MVector crossProduct(MVector a, MVector b);
    void showVoxel(vector<vector<vector<bool>>> grid, MString name);
    bool insideTriangleYZ(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double intersectTriangleX(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double edgeFunction(MPoint a, MPoint b, MVector p);
    void pyramidGCPU();
    void resetScaleField();
    void dilationCPU(MString SE, double baseScale);
    void connectedContourCPU();
    void scaleAugmentedPyramidCPU();
    template <typename T, typename CombineOp>
    vector<vector<vector<T>>> calculatePyramid(vector<vector<vector<T>>> prev, vector<MPoint> t, CombineOp combine);
    void erosionCPU();
    void cubeMarching();
    void createMayaMesh(MString name);
    void computeQuadricMatrices();
    double computeCollapseCost(Quadric& Q0, Quadric& Q1, Eigen::Vector3d& v_opt, int v0, int v1);
    void simplifyMesh();
    bool edgeLengthExceedsThreshold(int v0, int v1);
    void performCollapse(EdgeCollapse& ec);
};

#endif // BOUNDING_PROXY_H