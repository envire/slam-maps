#pragma once

#include "SurfacePatches.hpp"
#include "VoxelGridMap.hpp"
#include "MLSMap.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>
#include <cmath>
#include <base/TransformWithCovariance.hpp>

namespace maps { namespace grid
{

class TSDFVolumetricMap : public VoxelGridMap<TSDFPatch>
{
public:
    typedef boost::shared_ptr<TSDFVolumetricMap> Ptr;
    typedef const boost::shared_ptr<TSDFVolumetricMap> ConstPtr;
    typedef TSDFPatch VoxelCellType;
    typedef GridMap< DiscreteTree<VoxelCellType> > GridMapBase;
    typedef VoxelGridMap<VoxelCellType> VoxelGridBase;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    TSDFVolumetricMap(): VoxelGridMap<VoxelCellType>(Vector2ui::Zero(), Vector3d::Ones()),
                         truncation(1.f), min_varaince(0.001f) {}

    TSDFVolumetricMap(const Vector2ui &num_cells, const Vector3d &resolution, float truncation = 1.f, float min_varaince = 0.001f) :
                    VoxelGridMap<VoxelCellType>(num_cells, resolution), truncation(truncation), min_varaince(min_varaince) {}
    virtual ~TSDFVolumetricMap() {}

    void mergePointCloud(const PointCloud& pc, const base::Transform3d& pc2grid, double measurement_variance = 0.01);

    template<int _MatrixOptions>
    void mergePointCloud(const std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >& pc, const base::TransformWithCovariance& pc2grid,
                         const base::Vector3d& sensor_origin_in_pc = base::Vector3d::Zero(), double measurement_variance = 0.01);

    template<enum MLSConfig::update_model SurfaceType>
    void projectMLSMap(const maps::grid::MLSMap<SurfaceType>& mls, const base::Transform3d& mls2grid,
                       const Eigen::Vector2i& start_idx = Eigen::Vector2i(0,0),
                       const Eigen::Vector2i& end_idx = Eigen::Vector2i(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
                       float z_min = -50.f, float z_max = 50.f, float truncation = 1.f);

    void mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement, double measurement_variance = 0.01);

    bool hasSameFrame(const base::Transform3d& local_frame, const Vector2ui &num_cells, const Vector2d &resolution) const;

    void setTruncation(float truncation);

    float getTruncation();

    void setMinVariance(float min_varaince);

    float getMinVariance();

protected:

    /** truncation level of the signed distance function */
    float truncation;

    /** lower bound of the variance of each cell */
    float min_varaince;

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(VoxelGridMap<VoxelCellType>);
        ar & BOOST_SERIALIZATION_NVP(truncation);
        ar & BOOST_SERIALIZATION_NVP(min_varaince);
    }
};

template<int _MatrixOptions>
void TSDFVolumetricMap::mergePointCloud(const std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >& pc, const base::TransformWithCovariance& pc2grid,
                                        const base::Vector3d& sensor_origin_in_pc, double measurement_variance)
{
    Eigen::Vector3d sensor_origin_in_grid = pc2grid.getTransform() * sensor_origin_in_pc;

    for(typename std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >::const_iterator it = pc.begin(); it != pc.end(); ++it)
    {
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> measurement_in_map = pc2grid.composePointWithCovariance(*it, Eigen::Matrix3d::Zero());
        try
        {
            // TODO use variance in the direction of the measurement
            mergePoint(sensor_origin_in_grid, measurement_in_map.first, measurement_variance + measurement_in_map.second(2,2));
        }
        catch(const std::runtime_error& e)
        {
            // TODO use glog or base log for all out prints of this library
            std::cerr << e.what() << std::endl;
        }
    }
}

template<enum MLSConfig::update_model SurfaceType>
void TSDFVolumetricMap::projectMLSMap(const maps::grid::MLSMap<SurfaceType>& mls, const base::Transform3d& mls2grid,
                                      const Eigen::Vector2i& start_idx, const Eigen::Vector2i& end_idx,
                                      float z_min, float z_max, float truncation)
{
    base::Transform3d grid2mls = mls2grid.inverse();
    Eigen::Vector3d res = getVoxelResolution();
    Eigen::Vector3i max_idx;
    max_idx << end_idx.array().min(getNumCells().array().cast<int>()), (int)std::floor(z_max / res.z());
    Eigen::Vector3i min_idx;
    min_idx << start_idx, (int)std::floor(z_min / res.z());
    Eigen::Vector3i idx;
    Eigen::Vector3d cell_center;
    Eigen::Vector3d closest_point;
    for(idx.y() = min_idx.y(); idx.y() < max_idx.y(); idx.y() = idx.y() + 1)
    {
        for(idx.x() = min_idx.x(); idx.x() < max_idx.x(); idx.x() = idx.x() + 1)
        {
            for(idx.z() = min_idx.z(); idx.z() < max_idx.z(); idx.z() = idx.z() + 1)
            {
                if(fromVoxelGrid(idx, cell_center))
                {
                    cell_center = grid2mls * cell_center;
                    if(mls.getClosestContactPoint(cell_center, closest_point))
                    {
                        Eigen::Vector3d diff = (cell_center - closest_point);
                        float distance = diff.norm();
                        if(distance < truncation)
                        {
                            VoxelCellType& cell = getVoxelCell(idx);
                            cell.update(std::copysign(distance, diff.z()), 0.01, truncation, min_varaince);
                        }
                    }
                }
            }
        }
    }
}

}}