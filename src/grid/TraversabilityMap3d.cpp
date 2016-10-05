#include "TraversabilityMap3d.hpp"

namespace maps { namespace grid
{


TraversabilityNodeBase::TraversabilityNodeBase(float height, const Index &idx) :
    height(height), idx(idx), type(UNSET), mIsExpanded(false)
{

}

void TraversabilityNodeBase::addConnection(TraversabilityNodeBase* node)
{
    connections.push_back(node);
}

const std::vector< TraversabilityNodeBase* >& TraversabilityNodeBase::getConnections() const
{
    return connections;
}

float TraversabilityNodeBase::getHeight() const
{
    return height;
}

const Index& TraversabilityNodeBase::getIndex() const
{
    return idx;
}

void TraversabilityNodeBase::setHeight(float newHeight)
{
    height = newHeight;
}

bool TraversabilityNodeBase::operator<(const TraversabilityNodeBase& other) const
{
    return height < other.height;
}

bool TraversabilityNodeBase::isExpanded() const
{
    return mIsExpanded;
}

void TraversabilityNodeBase::setExpanded()
{
    mIsExpanded = true;
}

void TraversabilityNodeBase::setNotExpanded()
{
    mIsExpanded = false;
}

void TraversabilityNodeBase::setType(TraversabilityNodeBase::TYPE t)
{
    type = t;
}

TraversabilityNodeBase::TYPE TraversabilityNodeBase::getType() const
{
    return type;
}

TraversabilityNodeBase *TraversabilityNodeBase::getConnectedNode(const Index &toIdx) const
{
    for(maps::grid::TraversabilityNodeBase *con: connections)
    {
        if(toIdx == con->getIndex())
        {
            return con;
        }
    }
    return nullptr;
}


}}
