#ifndef __ENVIRE_MAPS_LOCAL_MAP_HPP__
#define __ENVIRE_MAPS_LOCAL_MAP_HPP__

#include <base/Eigen.hpp>

#include <boost/shared_ptr.hpp>

#include <string>

namespace envire { namespace maps
{
    /**@brief LocalMapType
     * The type of the LocalMap
     */

    // TODO: do we need unknown type?
    enum LocalMapType
    {
        GRID_MAP = 0,
        GEOMETRIC_MAP = 1,
        MLS_MAP = 2,
        TOPOLOGICAL_MAP = 3
    };

    struct LocalMapData
    {
        LocalMapData()
            : offset(base::Transform3d::Identity()) {};

        /**  string id of this local map **/
        std::string id;

        /** Offset within the grid. The description of the local map frame.
         * It will be the offset with respect
         * to the bottom left corner (origin) of the map.
         * For the time being we use 3D transformation.
         * 
         * 
         **/
        base::Transform3d offset;

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

    /**@brief LocalMap
     * Local map with respect to a given reference frame
     * A local map is the basic element to form a global
     * map (set of local maps structured in a tree).
     */
    class LocalMap
    {
        public:
            typedef boost::shared_ptr<LocalMap> Ptr;

            LocalMap()
                : data_ptr(new LocalMapData())
            {        
            }

            // TODO: which map_type should be set by default
            LocalMap(const boost::shared_ptr<LocalMapData> &data)
                : data_ptr(data)
            {
            }

            LocalMap(const LocalMap& other)
                : data_ptr(new LocalMapData(*(other.data_ptr.get())))
            {
            }

            virtual ~LocalMap() {};

            const std::string& getId() const
            {
                return data_ptr->id;
            }

            std::string& getId() 
            {
                return data_ptr->id;
            }            

            const base::Transform3d& getOffset() const
            {
                return data_ptr->offset;
            }

            base::Transform3d& getOffset()
            {
                return data_ptr->offset;
            }

        protected:
            boost::shared_ptr<LocalMapData> data_ptr;
    };
}}
#endif // __ENVIRE_MAPS_LOCAL_MAP_HPP__
