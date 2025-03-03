#include <maya/MFnPlugin.h>
#include <maya/MPxCommand.h>
#include <maya/MGlobal.h>
#include <iostream>
#include "boundingProxy.h"

MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, "MyPlugin", "1.0", "Any");

    status = plugin.registerCommand("BoundingProxyCmd", BoundingProxy::creator);
    if (!status) {
        status.perror("registerCommand");
        return status;
    }

    MString pluginPath = plugin.loadPath();
    MString scriptPath = pluginPath + "/../../scripts";

    MString melCmd =
        "global string $gMainWindow;\n"
        "if (!`menu -exists VoMoBoMenu`) {\n"
        "    menu -label \"VoMoBo\" -parent $gMainWindow VoMoBoMenu;\n"
        "}\n"
        "menuItem -label \"Simplify Mesh\" -parent VoMoBoMenu -command \"python(\\\"import sys; sys.path.append(r'" + scriptPath + "'); import gui; gui.show()\\\");\";";

    MGlobal::executeCommand(melCmd, true);

    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);

    status = plugin.deregisterCommand("generate");
    if (!status) {
        status.perror("deregisterCommand");
        return status;
    }

    MGlobal::executeCommand("if (`menu -exists VoMoBoMenu`) { deleteUI -menu VoMoBoMenu; }", true);

    return status;
}