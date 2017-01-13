#pragma once

#include <pcl/PolygonMesh.h>
#include "TSDFSurfaceReconstruction.hpp"

namespace maps { namespace tools
{

class TSDFPolygonMeshReconstruction : public TSDFSurfaceReconstruction<pcl::PolygonMesh>
{
public:
    TSDFPolygonMeshReconstruction() : TSDFSurfaceReconstruction<pcl::PolygonMesh>() {}

    void reconstruct(pcl::PolygonMesh &output);
};

}}