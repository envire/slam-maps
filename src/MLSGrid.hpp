#ifndef __ENVIRE_MAPS_MLS_GRID_HPP__
#define __ENVIRE_MAPS_MLS_GRID_HPP__

#include "GridMap.hpp"
#include "SPList.hpp"
#include "MLSConfig.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

namespace envire
{
    namespace maps 
    {
        // TODO move this typedef somewhere more globally
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

        class MLSGrid : public GridMap<SPList>
        {
        public:

            MLSGrid()
                : GridMap<SPList>()
            {};

            MLSGrid(const Vector2ui &num_cells, 
                    const Vector2d &resolution,
                    MLSConfig mls_config = MLSConfig())
                : GridMap<SPList>(num_cells, resolution, SPList(mls_config)),
                  mls_config(mls_config)
            {}

            const MLSConfig& getConfig() const 
            { 
                return mls_config; 
            }

            /**
             * Merges another MLSGrid into this. Both maps should have the same size and use the same configuration.
             */
            void merge(const MLSGrid& other);

            /**
             * Merges a pointcloud into this MLSGrid. The pointcloud contains the pose of its origin relative to the MLSGrid
             */
            void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = true);

        private:
            MLSConfig mls_config;


        };
    }
}

#endif // __ENVIRE_MAPS_MLS_GRID_HPP__
