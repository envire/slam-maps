#ifndef TRAVERSABILITYMAP3D_H
#define TRAVERSABILITYMAP3D_H

#include <list>
#include "GridMap.hpp"
#include "SurfacePatch.hpp"

namespace envire {
namespace maps {

class TraversabilityNode;

class NodeConnection
{
public:
    enum TYPE
    {
        OBSTACLE,
        TRAVERSABLE,
    };
    NodeConnection(TraversabilityNode *from, TraversabilityNode *to);
private:
    TraversabilityNode *from;
    TraversabilityNode *to;
};
    
class TraversabilityNode
{
    std::vector<NodeConnection *> connections;
    const SurfacePatch *corespondingPatch;
    envire::maps::Index idx;
public:
    TraversabilityNode(const SurfacePatch *patch);
    /**
     * Returns the index of this cell and the index of the
     * corresponding MLS Patch. The indices of the TraversabilityMap
     * and the MLSGrid must match at all times.
     * */
    const Index &getIndex() const;
    
    const SurfacePatch &getCorrespondingPatch() const;
    
    void addConnection(TraversabilityNode *node);
};
    
template <class T>
class TraversabilityNode2
{
    enum TYPE
    {
        OBSTACLE,
        TRAVERSABLE,
        UNKNOWN,
        HOLE,
    };
    std::vector<NodeConnection *> connections;
    const T *userData;
    envire::maps::Index idx;
    
    double height;
    
public:
    TraversabilityNode();
    TraversabilityNode(const T *userData);

    /**
     * Returns the index of this cell
     * */
    const Index &getIndex() const;
    
    const T &getUserData() const;
    
    void addConnection(TraversabilityNode *node);
};
    
class TraversabilityNodeList
{
    std::list<TraversabilityNode> nodeList;
 public:
    const std::list<TraversabilityNode> &getNodes();
    
    bool hasNodeforHeight(double height);
    TraversabilityNode *getNodeforHeight(double height);

    TraversabilityNode *getNodeforPatch(SurfacePatch *patch);
    
    //hm, pass the MLS Patch here ?
    TraversabilityNode *addNode(const SurfacePatch *patch, const Index &idx);
};

class TraversabilityMap3d : public envire::maps::GridMap<TraversabilityNodeList>
{
    
    
};

}
}

#endif // TRAVERSABILITYMAP3D_H
