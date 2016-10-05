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
        T userData;
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
    };

}}

