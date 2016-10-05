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

        void mergePatch(const Index &idx, const Patch& o)
        {
            CellType &list = Base::at(idx);

            typedef typename CellType::iterator iterator;
            iterator it = list.begin(), end = list.end(), it_prev = end;
            if(it==end)
            {
                // insert into empty list
                list.insert(o);
                return;
            }
            // Find iterators so that *it_prev < o < *it
            while(it != end && *it < o)
            {
                it_prev=it;
                ++it;
            }
            if(it_prev == end)
            {
                // new patch is smaller than first patch
                if(!merge(*it, o))
                    list.insert(o);
            }
            else if(it==end)
            {
                // new patch is larger than last patch
                if(!merge(*it_prev, o))
                    list.insert(o);
            }
            else
            {
                // new patch lies between it_prev and it
                // try to merge with previous patch:
                if(merge(*it_prev, o))
                {
                    // this might make this patch merge-able with the next patch
                    if(merge(*it_prev, *it))
                    {
                        // erase the second patch, since it was merged with the first
                        list.erase(it);
                    }
                }
                // otherwise, try to merge with the next patch
                else if(!merge(*it, o))
                {
                    // if it is not merge-able, insert as a new patch between existing patches
                    list.insert(it, o);
                }
            }

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
