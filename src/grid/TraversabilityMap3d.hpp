#ifndef TRAVERSABILITYMAP3D_H
#define TRAVERSABILITYMAP3D_H

#include <list>
#include "GridMap.hpp"
#include "SurfacePatches.hpp"
#include <map>

namespace maps {

class TraversabilityNodeBase
{
protected:
    std::vector<TraversabilityNodeBase *> connections;
    float height;
    ::maps::Index idx;
    
    
public:
    enum TYPE
    {
        OBSTACLE,
        TRAVERSABLE,
        UNKNOWN,
        HOLE,
    };

    TraversabilityNodeBase(float height, const Index &idx);

    float getHeight() const;
    
    /**
     * Returns the index of this cell
     * */
    const Index &getIndex() const;
    
    void addConnection(TraversabilityNodeBase *node);
    
    const std::vector<TraversabilityNodeBase *> &getConnections() const;
};

template <class T>
class TraversabilityNode : public TraversabilityNodeBase
{
    const T *userData;
public:
    TraversabilityNode(float height, const Index& idx ,const T *userDatap) : 
        TraversabilityNodeBase(height, idx), userData(userDatap) 
    {
    };

    const T &getUserData() const
    {
        return *userData;
    };
};
    
class TraversabilityNodeListBase
{
    typedef std::multimap<float, TraversabilityNodeBase *> List;
    List nodeList;
 public:
    TraversabilityNodeListBase();
    const List &getNodes() const;
    
    bool hasNodeforHeight(double height) const;
    void addNode(TraversabilityNodeBase *node)
    {
        nodeList.insert(std::make_pair(node->getHeight(), node));
    };
};

template <class T>
class TraversabilityNodeList
{
    TraversabilityNodeListBase *base;
public:

    TraversabilityNodeList(TraversabilityNodeListBase *base);
     
    const std::list<TraversabilityNodeBase *> &getNodes() const;

    bool hasNodeforHeight(double height) const;
    TraversabilityNode<T> *getNodeforHeight(double height) const;

    TraversabilityNode<T> *addNode(const T* userData, const Index &idx, float height)
    {
        TraversabilityNode<T> *newNode = new TraversabilityNode<T>(height, idx, userData);
        base->addNode(newNode);
    };
};


class TraversabilityMap3d : public ::maps::GridMap<TraversabilityNodeListBase>
{
public:
    Eigen::Vector3f getNodePosition(const TraversabilityNodeBase *node) const;
    
};

}

#endif // TRAVERSABILITYMAP3D_H
