#pragma once
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include <boost/noncopyable.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include "../src/TraversabilityMap3d.hpp"

namespace vizkit3d
{

class TraversabilityMap3dVisualization        
    : public vizkit3d::Vizkit3DPlugin<envire::maps::TraversabilityMap3d>
    , boost::noncopyable
{
    Q_OBJECT

protected:
    virtual void updateDataIntern(const envire::maps::TraversabilityMap3d& data);
    virtual void updateMainNode(osg::Node* node);
    
    virtual osg::ref_ptr< osg::Node > createMainNode();
    
    envire::maps::TraversabilityMap3d map;

    void addNodeList(const envire::maps::TraversabilityNodeListBase &l, osg::Group* group);
    
    void visualizeNode(const envire::maps::TraversabilityNodeBase *node);
    void visualizeConnection(const envire::maps::TraversabilityNodeBase *from, const envire::maps::TraversabilityNodeBase *to);
    
    osg::Group* nodeGroup;
    osg::Group* connectionGroup;
    
public:
    TraversabilityMap3dVisualization();
    virtual ~TraversabilityMap3dVisualization();
    
    Q_INVOKABLE void updateData(envire::maps::TraversabilityMap3d const &sample)
    {
        vizkit3d::Vizkit3DPlugin<envire::maps::TraversabilityMap3d>::updateData(sample);
    }

    

};

}
