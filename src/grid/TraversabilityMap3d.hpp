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
        void setHeight(float newHeight)
        {
            height = newHeight;
        }

        /**
         * Returns the index of this cell
         * */
        const Index &getIndex() const;

        void addConnection(TraversabilityNodeBase *node);

        const std::vector<TraversabilityNodeBase *> &getConnections() const;
        
        bool operator<(const TraversabilityNodeBase& other) const
        {
            return height < other.height;
        }
        
        bool isExpanded() const
        {
            return mIsExpanded;
        }
        
        void setExpanded()
        {
            mIsExpanded = true;
        }
        
        void setType(TYPE t)
        {
            type = t;
        }
        
        TYPE getType() const
        {
            return type;
        }

        double getDistToStart() const
        {
            return distToStart;
        }
        
        void setDistToStart(double newDist)
        {
            distToStart = newDist;
        }
        
    protected:
        std::vector<TraversabilityNodeBase *> connections;
        float height;
        ::maps::grid::Index idx;
        enum TYPE type;
        ///detemines wether this node is a candidate or a final node
        bool mIsExpanded;
    
        double distToStart;
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
    };

    template <class T>
    class TraversabilityMap3d : public ::maps::grid::MultiLevelGridMap<T>
    {
    public:
        TraversabilityMap3d() {};
        
        TraversabilityMap3d(const Index &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : MultiLevelGridMap<T>(num_cells, resolution, data), maxDist(0.0)
        {}

        Eigen::Vector3f getNodePosition(const TraversabilityNodeBase *node) const
        {
            Eigen::Vector3d pos;
            if(!this->fromGrid(node->getIndex(), pos))
                throw std::runtime_error("Internal error, could not calculate position from index");
            
            pos.z() += node->getHeight();
            
            return pos.cast<float>();
        }
        
        double maxDist;

    };

}}

