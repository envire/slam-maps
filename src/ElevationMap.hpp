#ifndef __ENVIRE_MAPS_ELEVATION_MAP_HPP__
#define __ENVIRE_MAPS_ELEVATION_MAP_HPP__

#include "GridMap.hpp"

namespace envire 
{
      namespace maps 
      {
            class ElevationMap : public GridMap
            {
            public:

                  static const std::string ELEVATION;
                  static const std::string ELEVATION_MIN;
                  static const std::string ELEVATION_MAX;

                  static const double ELEVATION_MAX_DEFAULT;
                  static const double ELEVATION_MIN_DEFAULT;

                  ElevationMap();

                  ElevationMap(GridConfig config);

                  ~ElevationMap();

                  Eigen::Vector3d getNormal( const Index& pos ) const;

                  /** @brief get the normal vector at the given position
                  */
                  Eigen::Vector3d getNormal( const Eigen::Vector2d& pos ) const;

                  /** @brief get the elevation at the given point 
                  *
                  * The underlying model assumes the height value to be at
                  * the center of the cell, and a surface is approximated
                  * using the getNormal. The Height value is the value of the
                  * plane at that point.
                  */
                  double getMeanElevation( const Eigen::Vector2d& pos ) const;

            };
      }
}


#endif // __ENVIRE_MAPS_ELEVATION_MAP_HPP__