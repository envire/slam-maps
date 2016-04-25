#ifndef __MAPS_MLS_GRID_HPP__
#define __MAPS_MLS_GRID_HPP__

#include <vector>

#include "GridMap.hpp"
#include "MLSConfig.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include "SurfacePatches.hpp"

#include <boost/scoped_ptr.hpp>

namespace base
{
namespace samples
{
struct Pointcloud;
} /* namespace samples */
} /* namespace base */

namespace maps { namespace grid
{

// TODO move this typedef somewhere more globally
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// TODO implement
class MLSMapExchange;

template<enum MLSConfig::update_model SPType>
struct MLSMapI;

typedef MLSMapI<MLSConfig::SLOPE> MLSMapSloped;
typedef MLSMapI<MLSConfig::KALMAN> MLSMapKalman;


} /* namespace grid */
} /* namespace maps */
#include "MLSMapI.hpp"

#endif // __MAPS_MLS_GRID_HPP__
