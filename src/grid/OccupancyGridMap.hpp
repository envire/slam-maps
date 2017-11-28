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
#pragma once

#include "OccupancyPatch.hpp"
#include "VoxelGridMap.hpp"
#include "OccupancyGridMapBase.hpp"
#include "OccupancyConfiguration.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>

namespace maps { namespace grid
{

class OccupancyGridMap : public OccupancyGridMapBase, public VoxelGridMap<OccupancyPatch>
{
public:
    typedef OccupancyPatch VoxelCellType;
    typedef GridMap< DiscreteTree<VoxelCellType> > GridMapBase;
    typedef VoxelGridMap<VoxelCellType> VoxelGridBase;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    OccupancyGridMap(): OccupancyGridMapBase(OccupancyConfiguration()),
                        VoxelGridMap<OccupancyPatch>(Vector2ui::Zero(), Vector3d::Ones()) {}

    OccupancyGridMap(const Vector2ui &num_cells, const Vector3d &resolution,
                    const OccupancyConfiguration& config) :
                    OccupancyGridMapBase(config),
                    VoxelGridMap<OccupancyPatch>(num_cells, resolution) {}
    virtual ~OccupancyGridMap() {}

    void mergePointCloud(const PointCloud& pc, const base::Transform3d& pc2mls);

    template<int _MatrixOptions>
    void mergePointCloud(const std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >& pc, const base::Transform3d& pc2grid,
                            const base::Vector3d& sensor_origin_in_pc = base::Vector3d::Zero())
    {
        Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin_in_pc;
        Eigen::Vector3i sensor_origin_idx;
        if(!VoxelGridBase::toVoxelGrid(sensor_origin_in_grid, sensor_origin_idx))
        {
            LOG_ERROR_S << "Sensor origin (" << sensor_origin_in_grid.transpose() << ") is outside of the grid! Can't add corresponding point cloud to grid.";
            return;
        }
        for(typename std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >::const_iterator it = pc.begin(); it != pc.end(); ++it)
        {
            try
            {
                mergePoint(sensor_origin_in_grid, sensor_origin_idx, pc2grid * (*it));
            }
            catch(const std::runtime_error& e)
            {
                LOG_ERROR_S << e.what();
            }
        }
    }

    void mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement);

    void mergePoint(const Eigen::Vector3d& sensor_origin, Eigen::Vector3i sensor_origin_idx, const Eigen::Vector3d& measurement);

    bool isOccupied(const Eigen::Vector3d& point) const;

    bool isOccupied(Index idx, float z) const;

    bool isFreeSpace(const Eigen::Vector3d& point) const;

    bool isFreeSpace(Index idx, float z) const;

    bool hasSameFrame(const base::Transform3d& local_frame, const Vector2ui &num_cells, const Vector2d &resolution) const;

protected:

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(OccupancyGridMapBase);
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(VoxelGridMap<OccupancyPatch>);
    }
};

}}

BOOST_CLASS_EXPORT_KEY(maps::grid::OccupancyGridMap);
