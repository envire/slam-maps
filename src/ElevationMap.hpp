#ifndef __ENVIRE_MAPS_ELEVATION_MAP_HPP__
#define __ENVIRE_MAPS_ELEVATION_MAP_HPP__

#include "GridMap.hpp"

namespace envire {namespace maps
{
    struct ElevationData 
    {
        static const double ELEVATION_DEFAULT;
        static const double ELEVATION_MIN_DEFAULT;

        /**
         * @brief [brief description]
         * @details Consturctor with default values
         */
        ElevationData() 
            : elevation(ELEVATION_DEFAULT),
              elevation_min(ELEVATION_MIN_DEFAULT)
        {}

        double elevation;
        double elevation_min;
    };

    class ElevationMap : public GridMap<ElevationData>
    {
    public:
        typedef boost::shared_ptr<ElevationMap> Ptr;

    public:
        ElevationMap();

        ElevationMap(const Vector2ui &num_cells, const Vector2d &resolution);

        ~ElevationMap();

        Vector3d getNormal(const Index& pos) const;

        /** @brief get the normal vector at the given position
        */
        Vector3d getNormal(const Vector3d& pos) const;

        /** @brief get the elevation at the given point 
        *
        * The underlying model assumes the height value to be at
        * the center of the cell, and a surface is approximated
        * using the getNormal. The Height value is the value of the
        * plane at that point.
        */
        double getMeanElevation(const Vector3d& pos) const;

        std::pair<double, double> getElevationRange() const;   

        std::pair<double, double> getElevationMinRange() const; 
    };
}}


#endif // __ENVIRE_MAPS_ELEVATION_MAP_HPP__
