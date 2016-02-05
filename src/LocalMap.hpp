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

    enum LocalMapType
    {
        UNKNOWN_MAP = -1,
        GRID_MAP = 1,
        GEOMETRIC_MAP = 2,
        TOPOLOGICAL_MAP = 3,
        MLS_MAP = 4
    };

    const std::string UNKNOWN_MAP_ID = "";
    const std::string DEFAULT_GRID_MAP_ID = "DEFAULT_GRID_MAP";
    const std::string UNKNOWN_EPSG_CODE = "NONE";

    struct LocalMapData
    {
        /** Grid map is the map by default **/
        LocalMapData()
            :id(UNKNOWN_MAP_ID),
            offset(base::Transform3d::Identity()),
            map_type(UNKNOWN_MAP),
            EPSG_code(UNKNOWN_EPSG_CODE){};

        LocalMapData(const std::string _id, const base::Transform3d &_offset,
                    const LocalMapType _map_type, const std::string _EPSG_code)
            :id(_id), offset(_offset), map_type(_map_type), EPSG_code(_EPSG_code) {};

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

            /**
             * @brief [brief description]
             * @details to share same content (LocalMapData) between various instance of LocalMap
             * 
             * @param data [description]
             */
            LocalMap(const boost::shared_ptr<LocalMapData> &data)
                : data_ptr(data)
            {
            }

            /**
             * @brief make copy without sharing the content
             * @details the copy instance owns a new content (LocalMapData)
             * 
             * @param other [description]
             */
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

            const boost::shared_ptr<LocalMapData>& getLocalMap() const
            {
                return data_ptr;
            }

        protected:
            boost::shared_ptr<LocalMapData> data_ptr;
    };
}}
#endif // __ENVIRE_MAPS_LOCAL_MAP_HPP__
