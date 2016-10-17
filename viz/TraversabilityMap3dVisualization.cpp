#include "TraversabilityMap3dVisualization.hpp"
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include "PatchesGeode.hpp"

using namespace ::maps::grid;
using namespace vizkit3d;

vizkit3d::TraversabilityMap3dVisualization::TraversabilityMap3dVisualization()
    : Vizkit3DPlugin< ::maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> >()
    , isoline_interval(16.0), show_connections(false)
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


void vizkit3d::TraversabilityMap3dVisualization::updateDataIntern(const maps::grid::TraversabilityMap3d< TraversabilityNodeBase* >& data)
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


void addSphere(const Eigen::Vector3f &position, osg::ref_ptr<osg::Geode> geode)
{
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(eigenVectorToOsgVec3(position.cast<double>()), 0.05);
    
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp);
    spd->setColor(osg::Vec4f(0, 1, 0, 1.0));
    geode->addDrawable(spd);
};


void TraversabilityMap3dVisualization::visualizeNode(const TraversabilityNodeBase* node)
{
    Eigen::Vector3f curNodePos = map.getNodePosition(node);
//     addSphere(curNodePos, nodeGeode);
//     nodeGroup->addChild(sphere);

    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setPosition(curNodePos.x(), curNodePos.y());
//     std::cout << "Drawing Plane " << std::endl;
    
//     curNodePos.z() -= node->getHeight();
    
    switch(node->getType())
    {
        case TraversabilityNodeBase::OBSTACLE:
            geode->setColor(osg::Vec4d(1,0,0,1));
            break;
        case TraversabilityNodeBase::UNKNOWN:
            geode->setColor(osg::Vec4d(1,0,1,1));
            break;
        case TraversabilityNodeBase::TRAVERSABLE:
        {
            geode->setColor(osg::Vec4d(0, 1, 0, 1));
            break;
        }
        default:
            geode->setColor(osg::Vec4d(0,0,1,1));
    }
    
    if(fabs(node->getHeight()) > 30000)
    {
        std::cout << "TraversabilityMap3dVisualization:: Warning, ignoring node with height above +-30000 " << std::endl; 
        return;
    }
    
    if(std::isnan(node->getHeight()))
        throw std::runtime_error("FOOOOOOOO");
    
    geode->drawPlane(node->getHeight(), 0.5, osg::Vec3d(0,0,0), osg::Vec3d(0,0,1));
    
    if(show_connections)
    {
        for(const TraversabilityNodeBase *conNode: node->getConnections())
        {
            visualizeConnection(node, conNode);
        }
    }
}

void TraversabilityMap3dVisualization::visualizeConnection(const TraversabilityNodeBase* from, const TraversabilityNodeBase* to)
{
    Eigen::Vector3f toPos = map.getNodePosition(to);
    Eigen::Vector3f fromPos = map.getNodePosition(from);
    osg::Vec3 fromOsg(fromPos.x(), fromPos.y(), fromPos.z());
    osg::Vec3 toOsg(toPos.x(), toPos.y(), toPos.z());
    
    linesNode->addLine(fromOsg, toOsg);
}

void vizkit3d::TraversabilityMap3dVisualization::addNodeList(const maps::grid::LevelList< TraversabilityNodeBase* >& l, osg::Group* group)
{
    for(const auto &entry: l)
    {
        visualizeNode(entry);
    }
}

void vizkit3d::TraversabilityMap3dVisualization::updateMainNode(osg::Node* node)
{
    osg::Group* group = static_cast<osg::Group*>(node);    

    nodeGroup = group;
    
    //clear old data
    group->removeChildren(0, group->getNumChildren());

    std::cout << "Map resolution " << map.getResolution().transpose() << std::endl;
//     nodeGeode = new osg::Geode();
    nodeGeode = new PatchesGeode(map.getResolution().x(), map.getResolution().y());
    linesNode = new osgviz::LinesNode(osg::Vec4(1, 1, 1, 1));
    group->addChild(nodeGeode);
    group->addChild(linesNode);

    
    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setColor(osg::Vec4d(1,0,0,1));
    geode->setShowPatchExtents(true);
    geode->setShowNormals(true);
    
//     addSphere(Eigen::Vector3f(0,0,0), geode);
    
    for(size_t y = 0; y < map.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < map.getNumCells().x(); x++)
        {
//             std::cout << "Cur Idx " << x << " " << y << std::endl;
            const LevelList<TraversabilityNodeBase *> &l(map.at(x, y));
            addNodeList(l, group);
        }
        
    }
    
//     geode->drawLines();
}

void vizkit3d::TraversabilityMap3dVisualization::setIsolineInterval(const double& val)
{
    isoline_interval = val;
    emit propertyChanged("isoline_interval");
    setDirty();
}

bool TraversabilityMap3dVisualization::getShowConnections()
{
    return show_connections;
}

void TraversabilityMap3dVisualization::setShowConnections(bool val)
{
    show_connections = true;
    emit propertyChanged("show_connections");
    setDirty();
}





