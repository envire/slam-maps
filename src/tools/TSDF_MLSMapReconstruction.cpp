#include "TSDF_MLSMapReconstruction.hpp"

using namespace maps::tools;
using namespace maps::grid;

void TSDF_MLSMapReconstruction::reconstruct(MLSMapPrecalculated& output)
{
    std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > surfaces;
    std::vector<float> intensities;
    reconstructSurfaces(surfaces, intensities, false);

    output = MLSMapPrecalculated(tsdf_map->getNumCells(), tsdf_map->getResolution(), MLSConfig());
    output.getLocalFrame().translation() << 0.5*tsdf_map->getResolution(), 0;

    Eigen::Vector3f p1, p2, p3;
    Index idx;
    Eigen::Vector3f center;
    Eigen::Vector3f normal;
    float min_z, max_z;
    for(unsigned i = 0; i < surfaces.size(); i += 3)
    {
        p1 = surfaces[i];
        p2 = surfaces[i+1];
        p3 = surfaces[i+2];
        normal = (p2 - p1).cross(p3 - p1);
        normal.normalize();

        // in case the vectors are zero or linearly dependent
        if(!normal.allFinite())
            continue;

        center = (p1 + p2 + p3) / 3.f;
        min_z = std::min(p1.z(), std::min(p2.z(), p3.z()));
        max_z = std::max(p1.z(), std::max(p2.z(), p3.z()));

        Eigen::Vector3d pos_diff;
        if(output.toGrid(center.cast<double>(), idx, pos_diff))
        {
            SurfacePatch< MLSConfig::PRECALCULATED > patch(pos_diff.cast<float>(), normal, min_z, max_z);
            // insert as new patch
            output.at(idx).insert(patch);
        }
        else
        {
            std::stringstream stream;
            stream << center.transpose();
            stream << " is outside of grid! This should never happen.";
            throw std::runtime_error(stream.str());
        }
    }

    // set local frame
    output.getLocalFrame() = output.getLocalFrame() * tsdf_map->getLocalFrame();
}