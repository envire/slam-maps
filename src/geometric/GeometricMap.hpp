#ifndef __MAPS_GEOMETRICMAP_HPP_
#define __MAPS_GEOMETRICMAP_HPP_

#pragma once

/** Maps classes **/
#include <maps/LocalMap.hpp>

/** Std vector **/
#include <vector>

/** Element type **/
#include <maps/geometric/GeometricElement.hpp>

namespace maps
{
    /**@brief GeometricMap class IEEE 1873 standard
     * This map is a Geometric structure for a vector metric (Cartesian) map
     * This map store collections of geometric entities (e.g. points, lines)
     */
    template <typename T >
    class GeometricMap: public LocalMap
    {

    private:
        std::vector< GeometricElement<T> > elements;

    public:
        GeometricMap()
            : LocalMap(maps::LocalMapType::GEOMETRIC_MAP)
        {
        }

        GeometricMap(const GeometricMap& other)
            : LocalMap(other)
        {
        }

        /** @brief default destructor
         */
        ~GeometricMap()
        {
        }

    public:

        size_t getNumElements() const
        {
            return this->elements.size();
        }



    };

}

#endif /* __MAPS_GEOMETRICMAP_HPP_ */
