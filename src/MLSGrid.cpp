#include "MLSGrid.hpp"

#include "tools/BresenhamLine.hpp"

#include <set>

namespace envire {
namespace maps {


void MLSGrid::mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty)
{
    // TODO change everything to float, if possible (requires refactoring Grid)

    const bool useNegative = mls_config.useNegativeInformation; // TODO also check that grid is not empty?
    MLSGrid tempGrid = useNegative
                    ? MLSGrid(this->getNumCells(), this->getResolution(), mls_config)
                    : MLSGrid();
    MLSGrid &workGrid = useNegative ? tempGrid : *this;

//    const std::vector<Eigen::Vector3d> & points = pc.vertices;
    // TODO uncertainty and color are ignored for now
    const double p_var = 0.01; // default uncertainty
    const double stdev = std::sqrt(p_var);
    typedef std::set<Index> IndexSet;
    IndexSet coveredCells;

    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        Eigen::Vector3d point = it->getArray3fMap().cast<double>();
        Eigen::Vector3d pos = pc2grid * point;
        Index idx;
        // TODO toGrid is expensive (involves applying another Transformation)
        if(toGrid(pos, idx, pos))
        {
            if(useNegative && at(idx).isCovered(pos.z(), stdev))
                continue;

            workGrid.at(idx).update(SurfacePatch(pos.cast<float>(), stdev));
            if(useNegative)
                coveredCells.insert(idx);
        }
    }
    if(useNegative)
    {
        Index origin; // Grid index of sensor
        Eigen::Vector3d orig_relative; // coordinates of sensor inside grid cell
        toGrid(pc2grid.translation(), origin, orig_relative);

        Bresenham::Point orig = origin.cast<int>(); // Origin in Bresenham compatible format
        for(IndexSet::const_iterator it = coveredCells.begin(); it != coveredCells.end(); ++it)
        {
            const SPList &cell = workGrid.at(*it);
            for(SPList::const_iterator cit = cell.begin(); cit != cell.end(); ++cit)
            {
                // Merge cell into main grid
                at(*it).update(*cit);

                // This block does actually the same for all cit:
                Bresenham bresLine(orig, it->cast<int>());
                Bresenham::Point diff_coord = it->cast<int>() - orig;
                Eigen::DenseIndex maxDim;
                diff_coord.cwiseAbs().maxCoeff(&maxDim);
                double invDiff = 1.0/diff_coord[maxDim];

                // get Range of individual patch:
                float min_z, max_z;
                cit->getRange(min_z, max_z);
                float z_max = max_z - orig_relative.z();
                float height = max_z - min_z;

                Bresenham::Point next;
                while(bresLine.getNextPoint(next))
                {
                    double factor = (next-orig)[maxDim] * invDiff;
                    if(factor <= 0 || factor >= 1.0) continue;
                    const double p_var = 0.05;
                    double z_mean = z_max * (factor) + orig_relative.z() - p_var;
                    double p_height = height * (factor) - 2*p_var;
                    double z_stdev = cit->getStdev() * factor;
                    SurfacePatch np(z_mean, z_stdev, p_height, SurfacePatch::NEGATIVE);
                    // TODO set update_idx of np?

                    // merge negative patch:
                    at(Index(next.cast<Index::Scalar>())).update(np);
                }
            }
        }
    }
}

}  // namespace maps
}  // namespace envire
