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
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
using namespace std;

constexpr double EPSILON = 1e-6;

class BoundingProxy : public MPxCommand {
public:
    MDagPath meshPath;
    MFnMesh* meshFn;
    vector<vector<vector<bool>>> G;
    MVector deltaP;
    unordered_map<int, vector<vector<vector<bool>>>> GHat;

    BoundingProxy();
    ~BoundingProxy();

    static void* creator();
    MStatus doIt(const MArgList&) override;
    MStatus SelectMesh();
    void VoxelizationCPU(int res);
    int World2Voxel(double w, double min, double max, int res);
    double Voxel2World(int v, double min, double max, int res);
    MVector CrossProduct(MVector a, MVector b);
    void ShowVoxel();
    bool InsideTriangleYZ(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double IntersectTriangleX(MPoint v0, MPoint v1, MPoint v2, double y, double z);
    double EdgeFunction(MPoint a, MPoint b, MVector p);
    void ClearAll();
    void MipMapCPU();
    vector<vector<vector<bool>>> CalculateGi(vector<vector<vector<bool>>> Gi_1, vector<vector<int>> t);
};

#endif // BOUNDING_PROXY_H