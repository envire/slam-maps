//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "OccupancyGridMap.hpp"
#include <boost/format.hpp>
#include <maps/tools/VoxelTraversal.hpp>

using namespace maps::grid;
using namespace maps::tools;

void OccupancyGridMap::mergePointCloud(const OccupancyGridMap::PointCloud& pc, const base::Transform3d& pc2grid)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin;
    
    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        try
        {
            Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
            mergePoint(sensor_origin_in_grid, pc2grid * measurement);
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << e.what();
        }
    }
}

void OccupancyGridMap::mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement)
{
    Eigen::Vector3i sensor_origin_idx;
    if(!VoxelGridBase::toVoxelGrid(sensor_origin, sensor_origin_idx))
    {
        throw std::runtime_error((boost::format("Sensor origin %1% is outside of the grid! Can't add to grid.") % sensor_origin.transpose()).str());
    }

    Eigen::Vector3i measurement_idx;
    if(VoxelGridBase::toVoxelGrid(measurement, measurement_idx))
    {
        std::vector<VoxelTraversal::RayElement> ray;
        VoxelTraversal::computeRay(VoxelGridBase::getVoxelResolution(), getLocalFrame() * sensor_origin, getLocalFrame() * measurement, ray);
        // remove last cell containing the measurement
        ray.pop_back();

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
