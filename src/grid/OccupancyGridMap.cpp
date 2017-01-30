#include "OccupancyGridMap.hpp"
#include <boost/format.hpp>
#include <maps/tools/VoxelTraversal.hpp>

using namespace maps::grid;
using namespace maps::tools;

void OccupancyGridMap::mergePointCloud(const OccupancyGridMap::PointCloud& pc, const base::Transform3d& pc2grid)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin;
    Eigen::Vector3i sensor_origin_idx;
    if(!VoxelGridBase::toVoxelGrid(sensor_origin_in_grid, sensor_origin_idx))
    {
        std::cerr << "Sensor origin (" << sensor_origin_in_grid.transpose() << ") is outside of the grid! Can't add corresponding point cloud to grid." << std::endl;
        return;
    }
    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        try
        {
            Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
            mergePoint(sensor_origin_in_grid, sensor_origin_idx, pc2grid * measurement);
        }
        catch(const std::runtime_error& e)
        {
            // TODO use glog or base log for all out prints of this library
            std::cerr << e.what() << std::endl;
        }
    }
}

void OccupancyGridMap::mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement)
{
    Eigen::Vector3i sensor_origin_idx;
    if(VoxelGridBase::toVoxelGrid(sensor_origin, sensor_origin_idx))
    {
        mergePoint(sensor_origin, sensor_origin_idx, measurement);
    }
    else
        throw std::runtime_error((boost::format("Sensor origin %1% is outside of the grid! Can't add to grid.") % sensor_origin.transpose()).str());
}

void OccupancyGridMap::mergePoint(const Eigen::Vector3d& sensor_origin, Eigen::Vector3i sensor_origin_idx, const Eigen::Vector3d& measurement)
{
    Eigen::Vector3i measurement_idx;
    if(VoxelGridBase::toVoxelGrid(measurement, measurement_idx))
    {
        std::vector<VoxelTraversal::RayElement> ray;
        VoxelTraversal::computeRay(VoxelGridBase::getVoxelResolution(), sensor_origin, sensor_origin_idx, measurement, ray);

        VoxelCellType& cell = getVoxelCell(measurement_idx);
        cell.updateLogOdds(config.hit_logodds, config.min_logodds, config.max_logodds);

        for(const VoxelTraversal::RayElement& element : ray)
        {
            DiscreteTree<VoxelCellType>& tree = at(element.idx);
            int32_t z_end = element.z_last + element.z_step;
            for(int32_t z_idx = element.z_first; z_idx != z_end; z_idx += element.z_step)
            {
                tree.getCellAt(z_idx).updateLogOdds(config.miss_logodds, config.min_logodds, config.max_logodds);
            }
        }
    }
    else
        throw std::runtime_error((boost::format("Point %1% or is outside of the grid! Can't add to grid.") % measurement.transpose()).str());
}

bool OccupancyGridMap::isOccupied(const Eigen::Vector3d& point) const
{
    Index idx;
    if(GridMapBase::toGrid(point, idx))
        return isOccupied(idx, point.z());
    throw std::runtime_error((boost::format("Point %1% is outside of the grid!") % point.transpose()).str());
}

bool OccupancyGridMap::isOccupied(Index idx, float z) const
{
    const DiscreteTree<VoxelCellType>& cell_tree = at(idx);
    DiscreteTree<VoxelCellType>::const_iterator it = cell_tree.find(z);
    return it != cell_tree.end() && it->second.getLogOdds() >= config.occupied_logodds;
}

bool OccupancyGridMap::isFreeSpace(const Eigen::Vector3d& point) const
{
    Index idx;
    if(GridMapBase::toGrid(point, idx))
        return isFreeSpace(idx, point.z());
    throw std::runtime_error((boost::format("Point %1% is outside of the grid!") % point.transpose()).str());
}

bool OccupancyGridMap::isFreeSpace(Index idx, float z) const
{
    const DiscreteTree<VoxelCellType>& cell_tree = at(idx);
    DiscreteTree<VoxelCellType>::const_iterator it = cell_tree.find(z);
    return it != cell_tree.end() && it->second.getLogOdds() <= config.free_space_logodds;
}

bool OccupancyGridMap::hasSameFrame(const base::Transform3d& local_frame, const Vector2ui& num_cells, const Vector2d& resolution) const
{
     if(getResolution() == resolution && getNumCells() == num_cells && getLocalFrame().isApprox(local_frame))
         return true;
     return false;
}

BOOST_CLASS_EXPORT_IMPLEMENT(maps::grid::OccupancyGridMap);
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
template void maps::grid::OccupancyGridMap::serialize(boost::archive::text_iarchive& arch, const unsigned int version);
template void maps::grid::OccupancyGridMap::serialize(boost::archive::text_oarchive& arch, const unsigned int version);
template void maps::grid::OccupancyGridMap::serialize(boost::archive::binary_iarchive& arch, const unsigned int version);
template void maps::grid::OccupancyGridMap::serialize(boost::archive::binary_oarchive& arch, const unsigned int version);