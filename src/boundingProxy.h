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
using namespace std;

class BoundingProxy : public MPxCommand {
public:
    double EPSILON = 1e-6;

    MDagPath meshPath;
    MFnMesh* meshFn;
    vector<vector<vector<bool>>> G;
    MVector deltaP;

    BoundingProxy();
    ~BoundingProxy();

    static void* creator();
    MStatus doIt(const MArgList&) override;
    MStatus SelectMesh();
    void Voxelization(int res);
    int World2Voxel(double w, double min, double max, int res);
    double Voxel2World(int v, double min, double max, int res);
    bool TriangleBoxOverlap(MPoint voxel, MPoint v0, MPoint v1, MPoint v2);
    MVector CrossProduct(MVector a, MVector b);
    bool PlaneBoxOverlap(MPoint p, MVector n, MPoint v0);
    bool ProjectionBoxOverlap(int axis1, int axis2, int axis3, vector<MPoint> v, MVector n, MPoint p);
    void ShowVoxel(int res);
};

#endif // BOUNDING_PROXY_H