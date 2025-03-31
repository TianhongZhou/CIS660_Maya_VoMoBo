#ifndef BRUSH_CONTEXT_H
#define BRUSH_CONTEXT_H
#include <maya/MPoint.h>
#include <maya/MEvent.h>
#include <maya/M3dView.h>
#include <maya/MGlobal.h>
#include <maya/MCursor.h>
#include <maya/MPxContextCommand.h>
#include <maya/MPxToolCommand.h>
#include <maya/MPxContext.h>
#include <maya/MSelectionMask.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

class BrushContext : public MPxContext {
public:
    MString mesh;
    double radius;
    double targetScale;
    MString mode;
    double baseScale;

    BrushContext();
    void toolOnSetup(MEvent& event) override;
    MStatus doPress(MEvent& event, MHWRender::MUIDrawManager& drawMgr, const MHWRender::MFrameContext& context) override;
    MStatus doDrag(MEvent& event, MHWRender::MUIDrawManager& drawMgr, const MHWRender::MFrameContext& context) override;
    void setTargetMesh(MString name);
    void setBrushRadius(double r);
    void setTargetScale(double ts);
    void setMode(MString m);
    void setBaseScale(double bs);
};

class BrushContextCmd : public MPxContextCommand {
public:
    BrushContext* contextPtr = nullptr;

    static void* creator();
    MPxContext* makeObj() override;
    MStatus doEditFlags() override;
    MStatus appendSyntax() override;
};

#endif // BRUSH_CONTEXT_H
