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
#include "TraversabilityMap3d.hpp"
#include <unordered_set>

namespace maps { namespace grid
{


TraversabilityNodeBase::TraversabilityNodeBase(float height, const Index &idx) :
    height(height), idx(idx), type(UNSET), mIsExpanded(false)
{

}

void TraversabilityNodeBase::eachConnectedNode(std::function<void (TraversabilityNodeBase *n, bool &explandNode, bool &stop)> f)
{
    std::deque<maps::grid::TraversabilityNodeBase*> nodes;
    std::unordered_set<const maps::grid::TraversabilityNodeBase*> visited;
    nodes.push_back(this);
    do
    {
        maps::grid::TraversabilityNodeBase* currentNode = nodes.front();
        nodes.pop_front();
        
        for(maps::grid::TraversabilityNodeBase* neighbor : currentNode->getConnections())
        {
            //check if we have already visited this node (happens because double connected graph)
            if(visited.find(neighbor) != visited.end())
                continue;

            visited.insert(neighbor);
                
            bool stop = false;
            bool expandNode = false;
            f(neighbor, expandNode, stop);
            
            if(stop)
            {
                return;
            }

            if(expandNode)
            {
                //further expand node
                nodes.push_back(neighbor);
            }
        }
    }while(!nodes.empty());
}

void TraversabilityNodeBase::eachConnectedNode(std::function<void (const TraversabilityNodeBase *n, bool &explandNode, bool &stop)> f) const
{
    std::deque<const maps::grid::TraversabilityNodeBase*> nodes;
    std::unordered_set<const maps::grid::TraversabilityNodeBase*> visited;
    nodes.push_back(this);
    do
    {
        const maps::grid::TraversabilityNodeBase* currentNode = nodes.front();
        nodes.pop_front();
        
        for(const maps::grid::TraversabilityNodeBase* neighbor : currentNode->getConnections())
        {
            //check if we have already visited this node (happens because double connected graph)
            if(visited.find(neighbor) != visited.end())
                continue;

            visited.insert(neighbor);
                
            bool stop = false;
            bool expandNode = false;
            f(neighbor, expandNode, stop);
            
            if(stop)
            {
                return;
            }

            if(expandNode)
            {
                //further expand node
                nodes.push_back(neighbor);
            }
        }
    }while(!nodes.empty());
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

Eigen::Vector3d TraversabilityNodeBase::getVec3(double grid_res) const
{
    return Eigen::Vector3d(idx.x() * grid_res, idx.y() * grid_res, height);
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
