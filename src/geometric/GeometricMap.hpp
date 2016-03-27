#ifndef __MAPS_GEOMETRICMAP_HPP_
#define __MAPS_GEOMETRICMAP_HPP_

#pragma once

/** Maps classes **/
#include <maps/LocalMap.hpp>

/** Std vector **/
#include <vector>
#include <unordered_map>
#include <typeindex>

/** Element type **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/LineSegment.hpp>

namespace maps
{
    /**@brief GeometricMap class IEEE 1873 standard
     * This map is a Geometric structure for a vector metric (Cartesian) map
     * This map store collections of geometric entities (e.g. points, lines)
     */
    class GeometricMap: public LocalMap
    {

    public:
        using ElementVector = std::vector<GeometricElementBase::Ptr>;
        using ElementMap = std::unordered_map<std::type_index, ElementVector>;

    private:
        /** Elements are stored by type.
         * Located with respect to the local frame (LocalMap)
         * **/
        ElementMap elements;

    public:

        typedef typename ElementMap::iterator iterator;
        typedef typename ElementMap::const_iterator const_iterator;

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

        inline iterator begin() { return this->elements.begin(); }

        inline iterator end() { return this->elements.end(); }

        size_t getNumElements() const
        {
            return this->elements.size();
        }

    };

}

#endif /* __MAPS_GEOMETRICMAP_HPP_ */
