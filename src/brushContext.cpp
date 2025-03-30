#include "brushContext.h"

BrushContext::BrushContext() {
    setTitleString("ScaleBrushContext");
    setCursor(MCursor::editCursor);
}

void BrushContext::toolOnSetup(MEvent& event) {}

MStatus BrushContext::doPress(MEvent& event, MHWRender::MUIDrawManager& drawMgr, const MHWRender::MFrameContext& context) {
    short x, y;
    event.getPosition(x, y);

    M3dView view = M3dView::active3dView();

    MPoint rayOrigin;
    MVector rayDir;
    view.viewToWorld(x, y, rayOrigin, rayDir);

    MString melCmd =
        "BoundingProxyCmd(\"edit_scale_field\", " +
        MString() + rayOrigin.x + ", " +
        MString() + rayOrigin.y + ", " +
        MString() + rayOrigin.z + ", " +
        MString() + rayDir.x + ", " +
        MString() + rayDir.y + ", " +
        MString() + rayDir.z + ", \"" +
        mesh + "\")";
    MGlobal::executeCommand(melCmd, true);

    return MS::kSuccess;
}

void BrushContext::setTargetMesh(MString name) {
    mesh = name;
}

void* BrushContextCmd::creator() {
    return new BrushContextCmd;
}

MPxContext* BrushContextCmd::makeObj() {
    contextPtr = new BrushContext();
    return contextPtr;
}

MStatus BrushContextCmd::appendSyntax() {
    MSyntax syntax = MPxContextCommand::syntax();
    syntax.addFlag("-m", "-mesh", MSyntax::kString);
    return MS::kSuccess;
}

MStatus BrushContextCmd::doEditFlags() {
    MArgParser argsDb = parser();

    if (argsDb.isFlagSet("-mesh")) {
        MString meshName;
        argsDb.getFlagArgument("-mesh", 0, meshName);

        if (contextPtr) {
            contextPtr->setTargetMesh(meshName);
        }
    }
    return MS::kSuccess;
}
