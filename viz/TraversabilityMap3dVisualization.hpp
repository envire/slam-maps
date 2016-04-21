#pragma once
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include <boost/noncopyable.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include "maps/grid/TraversabilityMap3d.hpp"

namespace vizkit3d
{

class TraversabilityMap3dVisualization        
    : public vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d>
    , boost::noncopyable
{
    Q_OBJECT

protected:
    virtual void updateDataIntern(const ::maps::grid::TraversabilityMap3d& data);
    virtual void updateMainNode(osg::Node* node);
    
    virtual osg::ref_ptr< osg::Node > createMainNode();
    
    ::maps::grid::TraversabilityMap3d map;

    void addNodeList(const ::maps::grid::TraversabilityNodeListBase &l, osg::Group* group);
    
    void visualizeNode(const ::maps::grid::TraversabilityNodeBase *node);
    void visualizeConnection(const ::maps::grid::TraversabilityNodeBase *from, const ::maps::grid::TraversabilityNodeBase *to);
    
    osg::Group* nodeGroup;
    osg::Group* connectionGroup;
    
public:
    TraversabilityMap3dVisualization();
    virtual ~TraversabilityMap3dVisualization();
    
    Q_INVOKABLE void updateData(::maps::grid::TraversabilityMap3d const &sample)
    {
        vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d>::updateData(sample);
    }

    

};

}
