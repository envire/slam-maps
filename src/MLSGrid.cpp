#include "MLSGrid.hpp"

#include "tools/BresenhamLine.hpp"

#include "GridMap.hpp"

#include "SPList.hpp"

#include <exception>
#include <set>


#include "MLSGridI.hpp"


namespace envire {

namespace maps {

#define ENVIRE_MAPS_MLSGRID(ret_type__) template<class SurfaceType> \
    ret_type__ MLSGridI<SurfaceType>

ENVIRE_MAPS_MLSGRID(void)::mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty)
    {
        // TODO change everything to float, if possible (requires refactoring Grid)

        const bool useNegative = false && mls_config.useNegativeInformation; // TODO also check that grid is not empty?
        MLSGridI tempGrid = useNegative
                        ? MLSGridI(grid.getNumCells(), grid.getResolution(), mls_config)
                        : MLSGridI();
        MLSGridI &workGrid = useNegative ? tempGrid : *this;

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
            if(grid.toGrid(pos, idx, pos))
            {
                if(useNegative && grid.at(idx).isCovered(pos.z(), stdev))
                    continue;

                workGrid.grid.at(idx).update(SurfaceType(pos.cast<float>(), stdev));
                if(useNegative)
                    coveredCells.insert(idx);
            }
        }
        if(useNegative)
        {
            Index origin; // Grid index of sensor
            Eigen::Vector3d orig_relative; // coordinates of sensor inside grid cell
            grid.toGrid(pc2grid.translation(), origin, orig_relative);

            Bresenham::Point orig = origin.cast<int>(); // Origin in Bresenham compatible format
            for(IndexSet::const_iterator it = coveredCells.begin(); it != coveredCells.end(); ++it)
            {
                const SPListST &cell = workGrid.grid.at(*it);
                for(typename SPListST::const_iterator cit = cell.begin(); cit != cell.end(); ++cit)
                {
                    // Merge cell into main grid
                    grid.at(*it).update(*cit);

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
                        //double z_stdev = cit->getStdev() * factor;
                        double z_stdev = 0.05 * factor; // TODO get stdev from patch
                        SurfaceType np(z_mean, z_stdev, p_height, SurfaceType::NEGATIVE);
                        // TODO set update_idx of np?

                        // merge negative patch:
                        grid.at(Index(next.cast<Index::Scalar>())).update(np);
                    }
                }
            }
        }
    }


ENVIRE_MAPS_MLSGRID(void)::mergePoint(const Vector3d & point)
{
    Eigen::Vector3d pos;
    Index idx;
    if(grid.toGrid(point, idx, pos))
    {
        // TODO get stddev from config or by function parameter
        grid.at(idx).update(SurfaceType(pos.cast<float>(), 0.1));
    }
}

MLSGrid::MLSGrid(
        const Vector2ui &num_cells_,
        const Eigen::Vector2d &resolution_,
        const MLSConfig & config)
{
    switch(config.updateModel)
    {
    case MLSConfig::SLOPE:
        map.reset(new MLSGridI<SurfacePatchT<MLSConfig::SLOPE> >(num_cells_, resolution_, config));
        break;
    case MLSConfig::KALMAN:
        map.reset(new MLSGridI<SurfacePatchT<MLSConfig::KALMAN> >(num_cells_, resolution_, config));
        break;
    default:
        throw std::runtime_error("Not implemented!");
        break;
    }
}
MLSGrid::MLSGrid()
{
    // empty
}

MLSGrid::~MLSGrid()
{
    // empty
}
MLSGrid::MLSGrid(const MLSGrid& other) : map(other.map->clone())
{ }

void MLSGrid::mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty)
{
    map->mergePointCloud(pc, pc2grid, withUncertainty);
}

void MLSGrid::mergePoint(const Eigen::Vector3d& point)
{
    map->mergePoint(point);
}


MLSGrid& MLSGrid::operator =(const MLSGrid& other)
{
    if(this != &other)
        map.reset(other.map->clone());
    return *this;
}

Eigen::Vector2d MLSGrid::getResolution() const
{
    return map->getResolution();
}
Eigen::Vector2d MLSGrid::getSize() const
{
    return map->getSize();
}
base::Transform3d& MLSGrid::getLocalFrame()
{
    return map->getLocalFrame();
}

template<class SPType>
const MLSGridI<SPType>& MLSGrid::getMLSGrid() const
{
    return dynamic_cast<const MLSGridI<SPType>&>(*map);
}
template<class SPType>
MLSGridI<SPType>& MLSGrid::getMLSGrid()
{
        return dynamic_cast<MLSGridI<SPType>&>(*map);
}

}  // namespace maps

}  // namespace envire

