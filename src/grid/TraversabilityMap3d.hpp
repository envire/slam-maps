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
#pragma once

#include <list>
#include "MultiLevelGridMap.hpp"
#include "SurfacePatches.hpp"
#include <map>

namespace maps { namespace grid
{

    class TraversabilityNodeBase
    {
    public:
        enum TYPE
        {
            OBSTACLE,
            TRAVERSABLE,
            UNKNOWN,
            HOLE,
            UNSET,
            FRONTIER, //a node that is traversable but is on the border to missing map information
        };

        TraversabilityNodeBase(float height, const Index &idx);

        float getHeight() const;
        void setHeight(float newHeight);

        /**
         * Returns the index of this cell
         * */
        const Index &getIndex() const;

        void addConnection(TraversabilityNodeBase *node);
        
        const std::vector<TraversabilityNodeBase *> &getConnections() const;
        
        TraversabilityNodeBase *getConnectedNode(const Index &toIdx) const;
        
        bool operator<(const TraversabilityNodeBase& other) const;
        
        bool isExpanded() const;
        void setExpanded();
        void setNotExpanded();
        
        void setType(TYPE t);
        TYPE getType() const;
        
    protected:
        TraversabilityNodeBase() {};
        
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            //we don't save the connections here, as this 
            //would lead to a stack corupption caused by 
            //to much recursive calls. The connections are
            //Saved and set in the map
            
            ar & height;
            ar & idx;
            ar & type;
            ar & mIsExpanded;
        }

        std::vector<TraversabilityNodeBase *> connections;
        float height;
        ::maps::grid::Index idx;
        enum TYPE type;
        ///detemines wether this node is a candidate or a final node
        bool mIsExpanded;
    };

    template <class T>
    class TraversabilityNode : public TraversabilityNodeBase
    {
    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;
        
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(TraversabilityNodeBase);
            ar & userData;
        }

        T userData;

        //needed for boost serialization
        TraversabilityNode() : TraversabilityNodeBase()
        {
        };
    public:
        TraversabilityNode(float height, const Index& idx) : 
            TraversabilityNodeBase(height, idx)
        {
        };
        
        T &getUserData()
        {
            return userData;
        };

        const T &getUserData() const
        {
            return userData;
        };
        
        TraversabilityNode<T> *getConnectedNode(const Index &toIdx) const
        {
            return static_cast<TraversabilityNode<T> *>(TraversabilityNodeBase::getConnectedNode(toIdx));
        }
    };

    template <class T>
    class TraversabilityMap3d : public ::maps::grid::MultiLevelGridMap<T>
    {
    public:
        TraversabilityMap3d() {};
        
        TraversabilityMap3d(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : MultiLevelGridMap<T>(num_cells, resolution, data)
        {}

        Eigen::Vector3f getNodePosition(const TraversabilityNodeBase *node) const
        {
            Eigen::Vector3d pos;
            if(!this->fromGrid(node->getIndex(), pos))
                throw std::runtime_error("Internal error, could not calculate position from index");
            
            pos.z() += node->getHeight();
            
            return pos.cast<float>();
        }
    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        struct SerializationHelper
        {
            T node;
            std::vector<TraversabilityNodeBase *> connections;
            
            /** Serializes the members of this class*/
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & node;
                ar & connections;
            }
        };
        
        /** Serializes the members of this class*/
        BOOST_SERIALIZATION_SPLIT_MEMBER()
        template<class Archive>
        void load(Archive &ar, const unsigned int version)
        {
            //load back all pointers
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(::maps::grid::MultiLevelGridMap<T>);

            //load nr of contained values
            uint64_t count;
            loadSizeValue(ar, count);
            
            for(size_t i=0; i<count; ++i)
            {
                SerializationHelper helper;
                ar >> helper;

                //set back the connections. As all pointers have been loaded before, this should
                //also apply to all objects in the MultiLevelGridMap.
                for(TraversabilityNodeBase *n : helper.connections)
                {
                    helper.node->addConnection(n);
                }
            }
        }
        template<class Archive>
        void save(Archive& ar, const unsigned int version) const
        {
            //first save all pointers without connections
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(::maps::grid::MultiLevelGridMap<T>);

            //determine nr of nodes
            uint64_t size = 0;
            for(const maps::grid::LevelList<T> &ll : *this )
            {
                for(const T &node: ll)
                {
                    size += ll.size();
                }
            }
            saveSizeValue(ar, size);

            //save all connections together with the coresponding pointer
            for(const maps::grid::LevelList<T> &ll : *this )
            {
                for(const T &node: ll)
                {
                    SerializationHelper helper;
                    helper.node = node;
                    helper.connections = node->getConnections();
                    
                    ar & helper;
                }
            }
        }
    };

    
    typedef TraversabilityMap3d<TraversabilityNodeBase *> TraversabilityBaseMap3d;
}}

