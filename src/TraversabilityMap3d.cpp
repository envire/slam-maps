#include "TraversabilityMap3d.hpp"

namespace envire {
    namespace maps {
        

Eigen::Vector3f TraversabilityMap3d::getNodePosition(const TraversabilityNodeBase* node) const
{
    Eigen::Vector3d pos;
    if(!fromGrid(node->getIndex(), pos))
        throw std::runtime_error("Internal error, could not calculate position from index");
    
    pos.z() += node->getHeight();
    
    return pos.cast<float>();
}

TraversabilityNodeBase::TraversabilityNodeBase(float height, const Index& idx) : height(height), idx(idx)
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

TraversabilityNodeListBase::TraversabilityNodeListBase()
{

}

const TraversabilityNodeListBase::List& TraversabilityNodeListBase::getNodes() const
{
    return nodeList;
}



}
}
