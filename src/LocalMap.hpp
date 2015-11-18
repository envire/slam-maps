#ifndef __ENVIRE_MAPS_LOCAL_MAP_HPP__
#define __ENVIRE_MAPS_LOCAL_MAP_HPP__


#include <base/TransformWithCovariance.hpp>

#include <boost/shared_ptr.hpp>

#include <string>

namespace envire { namespace maps
{
    /**@brief LocalMapType
     * The type of the LocalMap
     */
    enum LocalMapType
    {
        GRID_MAP = 0,
        GEOMETRIC_MAP = 1,
        MLS_MAP = 2,
        TOPOLOGICAL_MAP = 3
     };

    /**@brief LocalMap
     * Local map with respect to a given reference frame
     * A local map is the basic element to form a global
     * map (set of local maps structured in a tree).
     */
    class LocalMap
    {
        public:
            typedef boost::shared_ptr<LocalMap> Ptr;

        public:
            /**  string id of this local map **/
            std::string id;

            /** Offset of this local map with respect to the reference frame
             * This reference frame could be local or global. For example: in
             * case you are using this map with the envire_core, it will be
             * the offset with respect to the frame specified in the graph node. **/
            base::TransformWithCovariance offset;

            /** map_type of this local map **/
            LocalMapType map_type;

            /** EPSG_code that provides the geo-localized coordinate system **/
            std::string EPSG_code;

            /** The EPSG code depends in the coordinate system used for the
             * local map "NONE" in case of not geo-referenced map.
             * Example: "EPSG::4978" for the World Geodetic System 1984 (WGS-84)
             * Example: "EPSG::3199" Spain - Canary Islands onshore and offshore.
             * with bounds are [-21.93W, -11.75E] and [24.6S, 11.75N] using the
             * World Geodetic System 1984 (WGS-84) coordinate reference system
             * (CRS).
             * Example: "EPSG::5243" ETRS89 / LCC Germany (E-N) Bremen area with
             * WGS084 CRS
             * Reference: https://www.epsg-registry.org **/
    };
}}
#endif // __ENVIRE_MAPS_LOCALMAP_HPP__
