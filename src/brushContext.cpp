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
        mesh + "\", " +
        radius + ", " + 
        targetScale + ", \"" +
        mode + "\"" +
        ")";
    MGlobal::executeCommand(melCmd);

    return MS::kSuccess;
}

MStatus BrushContext::doDrag(MEvent& event, MHWRender::MUIDrawManager& drawMgr, const MHWRender::MFrameContext& context) {
    return doPress(event, drawMgr, context);
}

void BrushContext::setTargetMesh(MString name) {
    mesh = name;
}

void BrushContext::setBrushRadius(double r) {
    radius = r;
}

void BrushContext::setTargetScale(double ts) {
    targetScale = ts;
}

void BrushContext::setMode(MString m) {
    mode = m;
}

void BrushContext::setBaseScale(double bs) {
    baseScale = bs;
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
    syntax.addFlag("-r", "-radius", MSyntax::kDouble);
    syntax.addFlag("-ts", "-targetScale", MSyntax::kDouble);
    syntax.addFlag("-mo", "-mode", MSyntax::kString);
    syntax.addFlag("-bs", "-baseScale", MSyntax::kDouble);
    return MS::kSuccess;
}

MStatus BrushContextCmd::doEditFlags() {
    MArgParser argsDb = parser();

    if (argsDb.isFlagSet("-mesh")) {
        MString meshName;
        argsDb.getFlagArgument("-mesh", 0, meshName);
        if (contextPtr) contextPtr->setTargetMesh(meshName);
    }
    if (argsDb.isFlagSet("-radius")) {
        double radius;
        argsDb.getFlagArgument("-radius", 0, radius);
        if (contextPtr) contextPtr->setBrushRadius(radius);
    }
    if (argsDb.isFlagSet("-targetScale")) {
        double ts;
        argsDb.getFlagArgument("-targetScale", 0, ts);
        if (contextPtr) contextPtr->setTargetScale(ts);
    }
    if (argsDb.isFlagSet("-mode")) {
        MString mode;
        argsDb.getFlagArgument("-mode", 0, mode);
        if (contextPtr) contextPtr->setMode(mode);
    }
    if (argsDb.isFlagSet("-baseScale")) {
        double bs;
        argsDb.getFlagArgument("-baseScale", 0, bs);
        if (contextPtr) contextPtr->setBaseScale(bs);
    }
    return MS::kSuccess;
}
