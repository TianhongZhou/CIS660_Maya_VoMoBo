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
        selectMesh();

        // voxelization
        // closing
        // meshing
    }
    else if (command == "scale_field") {
        // beta version
    }
    else if (command == "show_voxel") {
        int resolution = args.asInt(1);
        
        selectMesh();

        // voxelization
        // show voxel (for debugging)
    }
    else {
        MGlobal::displayWarning("Unknown command argument!");
        return MS::kFailure;
    }

    return MS::kSuccess;
}

MStatus BoundingProxy::selectMesh() {
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