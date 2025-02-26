#include "boundingProxy.h"

void* BoundingProxy::creator() {
    return new BoundingProxy();
}

MStatus BoundingProxy::doIt(const MArgList&) {
    std::cout << "Generate called!" << std::endl;
    MGlobal::displayInfo("Generate called from Maya!");
    return MS::kSuccess;
}