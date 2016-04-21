#include "TraversabilityMap3dVisualization.hpp"
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

using namespace ::maps::grid;
using namespace vizkit3d;

vizkit3d::TraversabilityMap3dVisualization::TraversabilityMap3dVisualization(): Vizkit3DPlugin< ::maps::grid::TraversabilityMap3d >()
{

}

vizkit3d::TraversabilityMap3dVisualization::~TraversabilityMap3dVisualization()
{

}

osg::ref_ptr< osg::Node > vizkit3d::TraversabilityMap3dVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    return group.release();
}


void vizkit3d::TraversabilityMap3dVisualization::updateDataIntern(const ::maps::grid::TraversabilityMap3d& data)
{
    map = data;
}

void setColor(const osg::Vec4d& color, osg::Geode* geode)
{
    osg::Material *material = new osg::Material();
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    material->setShininess(osg::Material::FRONT, 10.0);

    geode->getOrCreateStateSet()->setAttribute(material);    
}


osg::ref_ptr<osg::Geode> getCylinder(const Eigen::Vector3f &from, const Eigen::Vector3f &to)
{
    Eigen::Vector3f diff = (to - from);
    Eigen::Vector3f center = from + diff * .5;
    
    osg::Quat quat; 
    osg::Matrix matrix; 
    matrix.makeLookAt(eigenVectorToOsgVec3(from.cast<double>()), eigenVectorToOsgVec3(to.cast<double>()), osg::X_AXIS); 
    quat.set(matrix); 
    
    osg::ref_ptr<osg::Geode> c2g = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> c2 = new osg::Cylinder(eigenVectorToOsgVec3(center.cast<double>()), 0.005, diff.norm());
    c2->setRotation(quat.inverse());
    osg::ref_ptr<osg::ShapeDrawable> c2d = new osg::ShapeDrawable(c2);
    c2g->addDrawable(c2d);
    setColor(osg::Vec4f(1.0, 0, 0, 1.0), c2g);
    
    return c2g;
}


osg::ref_ptr<osg::Geode> getSphere(const Eigen::Vector3f &position)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(eigenVectorToOsgVec3(position.cast<double>()), 0.01);
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp);
    spd->setColor(osg::Vec4f(0, 1, 0, 1.0));
    geode->addDrawable(spd);
    return geode;
};

void TraversabilityMap3dVisualization::visualizeNode(const TraversabilityNodeBase* node)
{
    Eigen::Vector3f curNodePos = map.getNodePosition(node);
    nodeGroup->addChild(getSphere(curNodePos));

    for(const TraversabilityNodeBase *conNode: node->getConnections())
    {
        visualizeConnection(node, conNode);
    }
}

void TraversabilityMap3dVisualization::visualizeConnection(const TraversabilityNodeBase* from, const TraversabilityNodeBase* to)
{
    Eigen::Vector3f toPos = map.getNodePosition(to);
    Eigen::Vector3f fromPos = map.getNodePosition(from);
    connectionGroup->addChild(getCylinder(fromPos, toPos));
}

void vizkit3d::TraversabilityMap3dVisualization::addNodeList(const TraversabilityNodeListBase& l, osg::Group* group)
{
    for(const auto &entry: l.getNodes())
    {
        visualizeNode(entry.second);
    }
}

void vizkit3d::TraversabilityMap3dVisualization::updateMainNode(osg::Node* node)
{
    osg::Group* group = static_cast<osg::Group*>(node);    

    //clear old data
    group->removeChildren(0, group->getNumChildren());

    for(size_t y = 0; y < map.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < map.getNumCells().x(); x++)
        {
            const TraversabilityNodeListBase &l(map.at(x, y));
            addNodeList(l, group);
        }
        
    }
    
}
