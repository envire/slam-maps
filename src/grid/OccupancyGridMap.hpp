#pragma once

#include "SurfacePatches.hpp"
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

    void mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement);

    void mergePoint(const Eigen::Vector3d& sensor_origin, Eigen::Vector3i sensor_origin_idx,
                    Eigen::Vector3d origin_cell_center, const Eigen::Vector3d& measurement);

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