//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef __MAPS_LOCAL_MAP_HPP__
#define __MAPS_LOCAL_MAP_HPP__

#include <base/Eigen.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <boost_serialization/EigenTypes.hpp>

#include <string>

namespace maps
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

            /** @brief Default constructor with only map type as argument */
            LocalMapData(const LocalMapType map_type)
                :id(UNKNOWN_MAP_ID),
                offset(base::Transform3d::Identity()),
                map_type(map_type),
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
             * @brief Offset of the local map frame
             * @details
             * The map uses the Cartesian coordinate system. Initially it is defined as follows:
             *    - the initial origin (0,0,0) of the map is placed in the bottom left corner
             *    - the x-axis points to the right from the origin
             *    - the y-axis points upwards from the origin
             *Y
             *^  0 1 2 3 4 5 6
             *|  0 1 2 3 4 5 6
             *|  0 1 2 3 4 5 6
             *|  0 1 2 3 4 5 6
             *|  0 1 2 3 4 5 6
             *O - - - - - - - -> X
             * 
             * The offset represents the displacement of the local frame
             * with respect to the initial coordinate system (grid frame).
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
     * the same LocalMapData content among various maps instances.
     * 
     * All maps, which share the same content of LocalMapData, own the same identification (id),
     * transformation of local frame (offset) and other parameters presented in LocalMapData. 
     * Therefore, the changes down on these parameters will affected all maps. 
     * 
     * To create a new LocalMap:
     *    - with its own LocalMapData content: 
     *    LocalMap() and LocalMap(const LocalMap& other)
     *    - with the LocalMapData content shared among different LocalMap instances: 
     *    LocalMap(const boost::shared_ptr<LocalMapData> &data).
     *    
     * Each local map has its own coordinate system or shares one coordinate system with other maps. 
     * The coordinate system is deifned by LocalMapData::offset, also named as local frame.
     * There is two opportunity to transform the coordinate system of the map:  
     *    - Through LocalMap::translate() and LocalMap::rotate() functions, to transform the map
     *    relative to its local frame. These function apply inverse of the given transformation
     *    to the local frame.
     *    - Through <a href="http://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html#a8fee784230fc83408b39716dc3fa9ab3" target="_blank">
     *    LocalMap::getLocalFrame().translate()</a>
     *     and <a href="http://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html#a2ed345ee2437ea5bfa0683bf2e82a3b0" target="_blank">
     *     LocalMap::getLocalFrame().rotate()</a>,
     *    to transform the local frame relative to itself.
     *    
     */
    class LocalMap
    {
        public:
            typedef boost::shared_ptr<LocalMap> Ptr;

            /**
             * @brief Default constructor to create a new LocalMapData content
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
             * @param other LocalMap
             */
            LocalMap(const LocalMap& other)
                : data_ptr(new LocalMapData(*(other.data_ptr.get())))
            {}

            /**
             * @brief Copy LocalMap and its content of LocalMapData without sharing it
             * @details 
             * The copy instance owns a new content (LocalMapData)
             * 
             * @param other LocalMap
             */
            LocalMap(const LocalMapType map_type)
                : data_ptr(new LocalMapData(map_type))
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
             * @brief Translate the map relative to its local frame
             * 
             * @param translation <a href="http://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga2006332f6989f501762673e21f5128f5" target="_blank">
             * Eigen::Vector3d</a>
             */
            void translate(const Eigen::Vector3d &translation)
            {
                data_ptr->offset.translate(-translation);
            }


            /**
             * @brief Return the translation of the grid
             */
            inline const Eigen::Vector3d translation() const
            {
                return data_ptr->offset.inverse().translation();
            }

            /**
             * @brief Rotate the map relative to its local frame
             * 
             * @param rotation <a href="http://eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga0d2bd45f1215359f8e7c0d7ab53c4acb" target="_blank">
             * Eigen::Quaterniond</a>
             */
            void rotate(const Eigen::Quaterniond &rotation)
            {
                data_ptr->offset.rotate(rotation.inverse());
            }

            /**
             * @brief Return the rotation of the grid
             */
            inline const Eigen::Quaterniond rotation() const
            {
                return Eigen::Quaterniond(data_ptr->offset.inverse().rotation());
            }


        private:
            boost::shared_ptr<LocalMapData> data_ptr;

            /** Grants access to boost serialization */
            friend class boost::serialization::access;

            /** Serializes the members of this class*/
            template <typename Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_NVP(data_ptr);
            }


    };
}
#endif // __MAPS_LOCAL_MAP_HPP__
