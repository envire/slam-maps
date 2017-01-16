#pragma once

#include <maps/grid/TSDFVolumetricMap.hpp>
#include <maps/grid/MLSMap.hpp>

#include "TSDFSurfaceReconstruction.hpp"

namespace maps { namespace tools
{

class TSDF_MLSMapReconstruction : public TSDFSurfaceReconstruction<maps::grid::MLSMapPrecalculated>
{
public:
    TSDF_MLSMapReconstruction() : TSDFSurfaceReconstruction<maps::grid::MLSMapPrecalculated>() {}

    void reconstruct(maps::grid::MLSMapPrecalculated &output);
};

}}