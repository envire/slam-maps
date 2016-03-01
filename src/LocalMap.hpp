#ifndef __ENVIRE_MAPS_LOCAL_MAP_HPP__
#define __ENVIRE_MAPS_LOCAL_MAP_HPP__

#include <base/Eigen.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost_serialization/EigenTypes.hpp>

#include <string>

namespace envire { namespace maps
{
    /** @brief The type of the LocalMap */
    enum LocalMapType
    {
        UNKNOWN_MAP = -1,
        GRID_MAP = 1,
        GEOMETRIC_MAP = 2,
        TOPOLOGICAL_MAP = 3,
        MLS_MAP = 4
    };

    const std::string UNKNOWN_MAP_ID = "";
    const std::string UNKNOWN_EPSG_CODE = "NONE";

    class LocalMapData
    {
        public:
            /** @brief Default constructor */
            LocalMapData()
                :id(UNKNOWN_MAP_ID),
                offset(base::Transform3d::Identity()),
                map_type(UNKNOWN_MAP),
                EPSG_code(UNKNOWN_EPSG_CODE) 
            {};

            /** @brief Constructor with parameters */
            LocalMapData(const std::string &id, const base::Transform3d &offset,
                        const LocalMapType map_type, const std::string &EPSG_code)
                :id(id), offset(offset), map_type(map_type), EPSG_code(EPSG_code) 
            {};

            /** @brief String id of the local map */
            std::string id;

            /** 
             * @brief Offest of the local map frame
             * @details
             * The map uses the Cartesian coordinate system:
             *    - the initial origin of the map is placed in the bottom left corner
             *    - the x-axis points to the right from the the origin
             *    - the y-axis points upwards from the origin
             * 
             * The offset represents the replacement of the local frame 
             * with respect to the inital origin.
             * 
             * The offsets along the x, y and z-axis (translation) are expressed in the map unit 
             * (according to the unit used in the map resolution).
             *
             * The offset can also be reference as local frame
             */
            base::Transform3d offset;

            /** @brief Type of the local map (s. #LocalMapType) */
            LocalMapType map_type;

            /** 
             * @brief EPSG_code that provides the geo-localized coordinate system 
             * @details
             * The EPSG code depends in the coordinate system used for the
             * local map "NONE" in case of not geo-referenced map.
             * Example: "EPSG::4978" for the World Geodetic System 1984 (WGS-84)
             * Example: "EPSG::3199" Spain - Canary Islands onshore and offshore.
             * with bounds are [-21.93W, -11.75E] and [24.6S, 11.75N] using the
             * World Geodetic System 1984 (WGS-84) coordinate reference system
             * (CRS).
             * Example: "EPSG::5243" ETRS89 / LCC Germany (E-N) Bremen area with
             * WGS084 CRS
             * Reference: https://www.epsg-registry.org 
             */
            std::string EPSG_code;

        private:
            /** Grants access to boost serialization */
            friend class boost::serialization::access;  

            /** Serializes the members of this class*/
            template <typename Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_NVP(id);
                ar & BOOST_SERIALIZATION_NVP(offset.matrix());
                ar & BOOST_SERIALIZATION_NVP(map_type);
                ar & BOOST_SERIALIZATION_NVP(EPSG_code);
            }                         
    };

    /**
     * @brief A class with basic map information
     * @details
     * A local map is the basic element to form a complex map, such as Grid.
     * 
     * Generally, this class is a holder for LocalMapData and was created to allow to share
     * the LocalMapData content (especially offset transformation) among various maps instances.
     * 
     * All maps, which share the same content of LocalMapData, own the same identification (id),
     * transformation of local frame (offset) and other parameters presented in LocalMapData. 
     * Therefore, the changes down on these parameters will affected all maps. 
     * 
     * To create a new LocalMap:
     *    - with its own LocalMapData content 
     *    (s. LocalMap() and LocalMap(const LocalMap& other)) 
     *    - share the same LocalMapData content among different LocalMap instances 
     *    (s. LocalMap(const boost::shared_ptr<LocalMapData> &data)).
     * 
     */
    class LocalMap
    {
        public:
            typedef boost::shared_ptr<LocalMap> Ptr;

            /**
             * @brief Default constructor to create a new LocalMapData content
             * @details [long description]
             * 
             * @param r [description]
             */
            LocalMap()
                : data_ptr(new LocalMapData())
            {}

            /**
             * @brief Constructor to share the same LocalMapData content
             * @details 
             * Use this constructor to share same content between various instance of LocalMap
             * 
             * @param data Shared pointer to LocalMapData instance
             */
            LocalMap(const boost::shared_ptr<LocalMapData> &data)
                : data_ptr(data)
            {}

            /**
             * @brief Copy LocalMap and its content of LocalMapData without sharing it
             * @details 
             * The copy instance owns a new content (LocalMapData)
             * 
             * @param other [description]
             */
            LocalMap(const LocalMap& other)
                : data_ptr(new LocalMapData(*(other.data_ptr.get())))
            {}

            virtual ~LocalMap() {};

            const std::string& getId() const
            {
                return data_ptr->id;
            }

            std::string& getId()
            {
                return data_ptr->id;
            }

            const base::Transform3d& getLocalFrame() const
            {
                return data_ptr->offset;
            }

            base::Transform3d& getLocalFrame()
            {
                return data_ptr->offset;
            }

            const LocalMapType& getMapType() const
            {
                return data_ptr->map_type;
            }

            LocalMapType& getMapType()
            {
                return data_ptr->map_type;
            }

            const std::string& getEPSGCode() const
            {
                return data_ptr->EPSG_code;
            }

            std::string& getEPSGCode()
            {
                return data_ptr->EPSG_code;
            }            

            const boost::shared_ptr<LocalMapData>& getLocalMapData() const
            {
                return data_ptr;
            }

            /**
             * @brief Translate the map with respect to its local frame
             * @details 
             * In the reality the local frame will be transform by the inverse translation
             * 
             * @param translation [description]
             */
            void translate(const Eigen::Vector3d &translation)
            {
                data_ptr->offset.translate(-translation);
            }

            /**
             * @brief Rotate the map with respect to its local frame
             * @details 
             * In the reality the local frame will be rotated by the inverse rotation
             * 
             * @param rotation [description]
             */
            void rotate(const Eigen::Quaterniond &rotation)
            {
                data_ptr->offset.rotate(rotation.inverse());
            }            

        protected:
            boost::shared_ptr<LocalMapData> data_ptr;
    };
}}
#endif // __ENVIRE_MAPS_LOCAL_MAP_HPP__
