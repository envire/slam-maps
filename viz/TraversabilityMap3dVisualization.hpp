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
    : public vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>>
    , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(double isoline_interval READ getIsolineInterval WRITE setIsolineInterval)

protected:
    virtual void updateDataIntern(const ::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>& data);
    virtual void updateMainNode(osg::Node* node);
    
    virtual osg::ref_ptr< osg::Node > createMainNode();
    
    ::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> map;

    void addNodeList(const ::maps::grid::LevelList<::maps::grid::TraversabilityNodeBase *> &l, osg::Group* group);
    
    void visualizeNode(const ::maps::grid::TraversabilityNodeBase *node);
    void visualizeConnection(const ::maps::grid::TraversabilityNodeBase *from, const ::maps::grid::TraversabilityNodeBase *to);
    
    osg::ref_ptr<osg::Geode> nodeGeode;
    
    osg::Group* nodeGroup;
    osg::Group* connectionGroup;
    
    double isoline_interval;

public:
    TraversabilityMap3dVisualization();
    virtual ~TraversabilityMap3dVisualization();
    
    Q_INVOKABLE void updateData(::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> const &sample)
    {
        vizkit3d::Vizkit3DPlugin<::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *>>::updateData(sample);
    }

    double getIsolineInterval() const { return isoline_interval; }
    void setIsolineInterval(const double& val);
};

}
