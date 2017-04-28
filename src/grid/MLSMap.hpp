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
#include <boost/serialization/shared_ptr.hpp>

#include "MultiLevelGridMap.hpp"
#include "MLSConfig.hpp"
#include "SurfacePatches.hpp"
#include "OccupancyGridMapBase.hpp"

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

        bool setFreeSpaceMap(boost::shared_ptr<OccupancyGridMapBase> free_space_map)
        {
            // check that size and resolution are the same
            if(free_space_map && free_space_map->hasSameFrame(Base::getLocalFrame(), Base::getNumCells(), Base::getResolution()))
            {
                this->free_space_map = free_space_map;
                return true;
            }
            return false;
        }

        boost::shared_ptr<OccupancyGridMapBase> getFreeSpaceMap() const
        {
            return free_space_map;
        }

        bool hasFreeSpaceMap() const
        {
            return free_space_map.get() != NULL;
        }

        bool getClosestContactPoint(const Vector3d& point, Vector3d& contact_point) const
        {
            Index idx;
            Vector3d pos_in_cell;
            if(Base::toGrid(point, idx, pos_in_cell))
            {
                Vector3 pos_in_cell_f = pos_in_cell.cast<float>();
                const CellType& cell = Base::at(idx);
                float min_dist;
                bool found_patch = false;
                Vector3d contact_point_in_cell;
                for(const Patch& patch : cell)
                {
                    Vector3 contact_point_f; // in local cell-coordinate system
                    float dist = std::abs(patch.getClosestContactPoint(pos_in_cell_f, contact_point_f));
                    if(found_patch && dist > min_dist)
                        break; // we already found a patch and the current patch is farer away. Since patches are sorted, we can't get closer
                    else
                    {
                        found_patch = true;
                        min_dist = dist;
                        contact_point_in_cell = contact_point_f.cast<double>();
                    }
                }
                if(found_patch && !base::isInfinity<float>(min_dist))
                {
                    Base::fromGrid(idx, contact_point, contact_point_in_cell, false);
                    return true;
                }
            }
            return false;
        }

        bool getClosestSurfacePos(const Vector3d& point, double& surface_pos) const
        {
            Index idx;
            Vector3d pos_in_cell;
            if(Base::toGrid(point, idx, pos_in_cell))
            {
                const CellType& cell = Base::at(idx);
                Vector3 pos_in_cell_f = pos_in_cell.cast<float>();
                float min_dist = base::infinity<float>();
                float cell_surface_pos = base::NaN<float>();
                for(const Patch& patch : cell)
                {
                    float surface_pos_f = patch.getSurfacePos(pos_in_cell_f);
                    float dist = std::abs(surface_pos_f - pos_in_cell_f.z());
                    if(dist > min_dist)
                        break;
                    else
                    {
                        min_dist = dist;
                        cell_surface_pos = surface_pos_f;
                    }
                }
                if(!base::isInfinity<float>(min_dist))
                {
                    // transform from grid to map frame
                    pos_in_cell.z() = cell_surface_pos;
                    Vector3d surface_in_map;
                    Base::fromGrid(idx, surface_in_map, pos_in_cell, false);
                    surface_pos = surface_in_map.z();
                    return true;
                }
            }
            return false;
        }

        void mergeMLS(const MLSMap& other)
        {
            // TODO implement
            throw std::runtime_error("mergeMLS is not yet implemented!");
        }

        void mergePointCloud(const PointCloud& pc, const base::Transform3d& pc2mls, double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls);
            if(hasFreeSpaceMap())
            {
                Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
                Eigen::Vector3d sensor_origin_in_mls = pc2mls * sensor_origin;
                for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
                {
                    Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
                    Eigen::Vector3d measurement_in_map = pc2mls * measurement;

                    try
                    {
                        if(!free_space_map->isFreeSpace(measurement_in_map))
                            mergePoint(measurement, pc2grid, measurement_variance);

                        free_space_map->mergePoint(sensor_origin_in_mls, measurement_in_map);
                    }
                    catch(const std::runtime_error& e)
                    {
                        // TODO use glog or base log for all out prints of this library
                        std::cerr << e.what() << std::endl;
                    }
                }
            }
            else
            {
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
        }

        void mergePointCloud(const PointCloud& pc, const base::TransformWithCovariance& pc2mls, double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls.getTransform());
            if(hasFreeSpaceMap())
            {
                Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
                Eigen::Vector3d sensor_origin_in_mls = pc2mls.getTransform() * sensor_origin;
                for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
                {
                    Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> measurement_in_map = pc2mls.composePointWithCovariance(measurement, Eigen::Matrix3d::Zero());

                    try
                    {
                        if(!free_space_map->isFreeSpace(measurement_in_map.first))
                            mergePoint(measurement, pc2grid, measurement_variance + measurement_in_map.second(2,2));

                        if(measurement_in_map.second(2,2) <= free_space_map->getConfig().uncertainty_threshold)
                            free_space_map->mergePoint(sensor_origin_in_mls, measurement_in_map.first);
                    }
                    catch(const std::runtime_error& e)
                    {
                        // TODO use glog or base log for all out prints of this library
                        std::cerr << e.what() << std::endl;
                    }
                }
            }
            else
            {
                for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
                {
                    Eigen::Vector3d point = it->getArray3fMap().cast<double>();
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> point_with_cov = pc2mls.composePointWithCovariance(point, Eigen::Matrix3d::Zero());
                    try
                    {
                        mergePoint(point, pc2grid, measurement_variance + point_with_cov.second(2,2));
                    }
                    catch(const std::runtime_error& e)
                    {
                        // TODO use glog or base log for all out prints of this library
                        std::cerr << e.what() << std::endl;
                    }
                }
            }
        }

        template<int _MatrixOptions>
        void mergePointCloud(const std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >& pc, const base::TransformWithCovariance& pc2mls,
                             const base::Vector3d& sensor_origin_in_pc = base::Vector3d::Zero(), double measurement_variance = 0.01)
        {
            base::Transform3d pc2grid = Base::prepareToGridOptimized(pc2mls.getTransform());
            if(hasFreeSpaceMap())
            {
                base::Vector3d sensor_origin_in_mls = pc2mls.getTransform() * sensor_origin_in_pc;
                for(typename std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >::const_iterator it = pc.begin(); it != pc.end(); ++it)
                {
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> measurement_in_map = pc2mls.composePointWithCovariance(*it, Eigen::Matrix3d::Zero());

                    try
                    {
                        if(!free_space_map->isFreeSpace(measurement_in_map.first))
                            mergePoint(*it, pc2grid, measurement_variance + measurement_in_map.second(2,2));

                        if(measurement_in_map.second(2,2) <= free_space_map->getConfig().uncertainty_threshold)
                            free_space_map->mergePoint(sensor_origin_in_mls, measurement_in_map.first);
                    }
                    catch(const std::runtime_error& e)
                    {
                        // TODO use glog or base log for all out prints of this library
                        std::cerr << e.what() << std::endl;
                    }
                }
            }
            else
            {
                for(typename std::vector< Eigen::Matrix<double, 3, 1, _MatrixOptions> >::const_iterator it = pc.begin(); it != pc.end(); ++it)
                {
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> point_with_cov = pc2mls.composePointWithCovariance(*it, Eigen::Matrix3d::Zero());
                    try
                    {
                        mergePoint(*it, pc2grid, measurement_variance + point_with_cov.second(2,2));
                    }
                    catch(const std::runtime_error& e)
                    {
                        // TODO use glog or base log for all out prints of this library
                        std::cerr << e.what() << std::endl;
                    }
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
        boost::shared_ptr<OccupancyGridMapBase> free_space_map;

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
        template<class Archive>
        void save(Archive & ar, const unsigned int version) const
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MultiLevelGridMap<SurfacePatch<SurfaceType>>);
            ar & BOOST_SERIALIZATION_NVP(config);
            ar & BOOST_SERIALIZATION_NVP(free_space_map);
        }

        template<class Archive>
        void load(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MultiLevelGridMap<SurfacePatch<SurfaceType>>);
            ar & BOOST_SERIALIZATION_NVP(config);
            if(version >= 1)
                ar & BOOST_SERIALIZATION_NVP(free_space_map);
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()
    };

    typedef MLSMap<MLSConfig::SLOPE> MLSMapSloped;
    typedef MLSMap<MLSConfig::KALMAN> MLSMapKalman;
    typedef MLSMap<MLSConfig::PRECALCULATED> MLSMapPrecalculated;

} /* namespace grid */
} /* namespace maps */

BOOST_CLASS_VERSION(maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>, 1)
BOOST_CLASS_VERSION(maps::grid::MLSMap<maps::grid::MLSConfig::KALMAN>, 1)
BOOST_CLASS_VERSION(maps::grid::MLSMap<maps::grid::MLSConfig::PRECALCULATED>, 1)

#endif // __MAPS_MLS_GRID_HPP__
