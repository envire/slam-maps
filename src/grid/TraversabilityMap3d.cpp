#include "TraversabilityMap3d.hpp"

namespace maps { namespace grid
{


TraversabilityNodeBase::TraversabilityNodeBase(float height, const Index &idx) : height(height), idx(idx), type(UNSET), mIsExpanded(false), distToStart(0.0)
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

}}
