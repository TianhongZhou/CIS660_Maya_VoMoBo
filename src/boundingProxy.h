#ifndef BOUNDING_PROXY_H
#define BOUNDING_PROXY_H
#include <maya/MPxCommand.h>
#include <maya/MGlobal.h>
#include <iostream>

class BoundingProxy : public MPxCommand {
public:
    static void* creator();

    MStatus doIt(const MArgList&) override;
};

#endif // BOUNDING_PROXY_H