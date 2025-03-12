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
#include <igl/copyleft/marching_cubes.h>
#include <Eigen/Dense>
using namespace std;

constexpr double EPSILON = 1e-6;

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

    BoundingProxy();
    ~BoundingProxy();

    static void* creator();
    MStatus doIt(const MArgList&) override;
    MStatus SelectMesh();
    void VoxelizationCPU(int res);
    int World2Voxel(double w, double min, double max, int res);
    double Voxel2World(int v, double min, double max, int res);
    MVector CrossProduct(MVector a, MVector b);
    void ShowVoxel(vector<vector<vector<bool>>> grid, MString name);
    bool InsideTriangleYZ(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double IntersectTriangleX(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double EdgeFunction(MPoint a, MPoint b, MVector p);
    void PyramidGCPU();
    void ResetScaleField();
    void DilationCPU(MString SE, double baseScale);
    void ConnectedContourCPU();
    void ScaleAugmentedPyramidCPU();
    template <typename T, typename CombineOp>
    vector<vector<vector<T>>> CalculatePyramid(vector<vector<vector<T>>> prev, vector<MPoint> t, CombineOp combine);
    void ErosionCPU();
    void CubeMarching();
    void CreateMayaMesh(MString name);
};

#endif // BOUNDING_PROXY_H