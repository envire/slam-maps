#ifndef __MAPS_MLS_GRID_HPP__
#define __MAPS_MLS_GRID_HPP__

#include <vector>
#include <set>
#include <exception>

#include <Eigen/Geometry>

#include <boost/scoped_ptr.hpp>
#include <boost/format.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

#include "MultiLevelGridMap.hpp"
#include "MLSConfig.hpp"
#include "SurfacePatches.hpp"

#include <maps/tools/BresenhamLine.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <base/TransformWithCovariance.hpp>


namespace maps { namespace grid
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    template<enum MLSConfig::update_model  SurfaceType>
    class MLSMap : public MultiLevelGridMap<SurfacePatch<SurfaceType> >
    {
        public:
            typedef SurfacePatch<SurfaceType> Patch;
            typedef MultiLevelGridMap<Patch> Base;
            typedef LevelList<Patch> CellType; 

        MLSMap(
                const Vector2ui &num_cells,
                const Vector2d &resolution,
                const MLSConfig &config_)
        : Base(num_cells, resolution)
        , config(config_)
        {
            // TODO assert that config is compatible to SurfaceType ...
        }

        MLSMap()
        {
            // empty
        }

        template<enum MLSConfig::update_model OtherSurfaceType>
        MLSMap(const MLSMap<OtherSurfaceType>& other) : Base(other)
        {

        }

        const MLSConfig& getConfig() const
        {
            return config;
        }

        void mergeMLS(const MLSMap& other)
        {
            // TODO implement
            throw std::runtime_error("mergeMLS is not yet implemented!");
        }

        void mergePointCloud(const PointCloud& pc, const base::Transform3d& pc2mls, double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls);
            for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
            {
                try
                {
                    mergePoint(it->getArray3fMap().cast<double>(), pc2grid, measurement_variance);
                }
                catch(const std::runtime_error& e)
                {
                    // TODO use glog or base log for all out prints of this library
                    std::cerr << e.what() << std::endl;
                }
            }
        }

        void mergePointCloud(const PointCloud& pc, const base::TransformWithCovariance& pc2mls, double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls.getTransform());
            for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
            {
                try
                {
                    Eigen::Vector3d point = it->getArray3fMap().cast<double>();
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> point_with_cov = pc2mls.composePointWithCovariance(point, Eigen::Matrix3d::Zero());
                    mergePoint(point, pc2grid, measurement_variance + point_with_cov.second(2,2));
                }
                catch(const std::runtime_error& e)
                {
                    // TODO use glog or base log for all out prints of this library
                    std::cerr << e.what() << std::endl;
                }
            }
        }

        template<int _MatrixOptions>
        void mergePointCloud(const std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >& pc,
                             const base::TransformWithCovariance& pc2mls, double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls.getTransform());
            for(typename std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >::const_iterator it = pc.begin(); it != pc.end(); ++it)
            {
                try
                {
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> point_with_cov = pc2mls.composePointWithCovariance(*it, Eigen::Matrix3d::Zero());
                    mergePoint(*it, pc2grid, measurement_variance + point_with_cov.second(2,2));
                }
                catch(const std::runtime_error& e)
                {
                    // TODO use glog or base log for all out prints of this library
                    std::cerr << e.what() << std::endl;
                }
            }
        }

        void mergePatch(const Index &idx, const Patch& new_patch)
        {
            CellType &list = Base::at(idx);

            for(typename CellType::iterator patch_it = list.begin(); patch_it != list.end(); patch_it++)
            {
                // test if it can be merged with an existing patch
                if(merge(*patch_it, new_patch))
                {
                    // since patch_it was changed test if it can be merged with any of the existing patches
                    mergePatchRecursive(list, patch_it);
                    return;
                }
                else if(new_patch < *patch_it)
                    break;
            }
            // insert as new patch
            list.insert(new_patch);
        }

        void mergePoint(const Eigen::Vector3d& point, double measurement_variance = 0.01)
        {
            Eigen::Vector3d pos_diff;
            Index idx;
            if(Base::toGrid(point, idx, pos_diff))
                mergePatch(idx, Patch(pos_diff.cast<float>(), measurement_variance));
            else
                throw std::runtime_error((boost::format("Point %1% is outside of the grid! Can't add to grid.") % point.transpose()).str());
        }

        /**
         * Adds a point with the given transformation to the grid.
         * Note: Use \c prepareToGridOptimized to prepare the pc2gridframe transformation.
         * The measurement variance is the uncertainty on the z axis of the point.
         */
        void mergePoint(const Eigen::Vector3d& point, const base::Transform3d& pc2gridframe, double measurement_variance = 0.01)
        {
            Eigen::Vector3d pos_diff;
            Index idx;
            if(Base::toGridOptimized(point, idx, pos_diff, pc2gridframe))
            {
                mergePatch(idx, Patch(pos_diff.cast<float>(), measurement_variance));
            }
            else
                throw std::runtime_error((boost::format("Point %1% is outside of the grid! Can't add to grid.") % point.transpose()).str());
        }

    private:
        MLSConfig config;

        bool merge(Patch& a, const Patch& b)
        {
            return a.merge(b, config);
        }

        bool isCovered(const Index &idx, float zPos, const float gapSize = 0.0)
        {
            CellType &list = Base::at(idx);

            for(typename CellType::const_iterator it = list.begin(); it!= list.end(); ++it)
            {
                if(it->isCovered(zPos, gapSize)) return true;
            }
            return false;
        }

        /**
         * Checks if a existing patch in the current cell list can be merged with any of the other patches.
         * Recursively resolves all possible following merge tasks.
         * Note: Since the CellType is a boost::flat_set which invalidates all following iterators if an element
         * is erased the implementation has to take care of the order in which patches are merged.
         */
        void mergePatchRecursive(CellType& list, typename CellType::iterator& patch)
        {
            for(typename CellType::iterator patch_it = list.begin(); patch_it != list.end(); patch_it++)
            {
                if(patch_it == patch)
                    continue;
                else if(patch_it < patch && merge(*patch_it, *patch))
                {
                    patch = list.erase(patch);
                    patch = list.end();
                    mergePatchRecursive(list, patch_it);
                    return;
                }
                else if(patch < patch_it)
                {
                    if(merge(*patch, *patch_it))
                    {
                        patch_it = list.erase(patch_it);
                        mergePatchRecursive(list, patch);
                    }
                    return;
                }
            }
        }

    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MultiLevelGridMap<SurfacePatch<SurfaceType>>);
            ar & BOOST_SERIALIZATION_NVP(config);
        }         
    };

    typedef MLSMap<MLSConfig::SLOPE> MLSMapSloped;
    typedef MLSMap<MLSConfig::KALMAN> MLSMapKalman;
    typedef MLSMap<MLSConfig::PRECALCULATED> MLSMapPrecalculated;

} /* namespace grid */
} /* namespace maps */

#endif // __MAPS_MLS_GRID_HPP__
