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
#include "TraversabilityMap3dVisualization.hpp"
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include "PatchesGeode.hpp"

using namespace ::maps::grid;
using namespace vizkit3d;

vizkit3d::TraversabilityMap3dVisualization::TraversabilityMap3dVisualization()
    : MapVisualization< maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > >()
    , isoline_interval(16.0)
    , show_connections(false)
{

}

vizkit3d::TraversabilityMap3dVisualization::~TraversabilityMap3dVisualization()
{

}

osg::ref_ptr< osg::Node > vizkit3d::TraversabilityMap3dVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();
    localNode = new osg::Group();

    mainNode->addChild(localNode.get());

    return mainNode;
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
    Eigen::Vector2f curNodePos = (node->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();

    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setPosition(curNodePos.x(), curNodePos.y());
    
    switch(node->getType())
    {
        case TraversabilityNodeBase::OBSTACLE:
            geode->setColor(osg::Vec4d(1,0,0,1));
            break;
        case TraversabilityNodeBase::UNKNOWN:
            geode->setColor(osg::Vec4d(1,0,1,1));
            break;
        case TraversabilityNodeBase::TRAVERSABLE:
            geode->setColor(osg::Vec4d(0, 1, 0, 1));
            break;
        case TraversabilityNodeBase::FRONTIER:
            geode->setColor(osg::Vec4d(0, 0, 1, 1));
            break;
        case TraversabilityNodeBase::HOLE:
            geode->setColor(osg::Vec4d(0, 1, 1, 1));
            break;
        case TraversabilityNodeBase::UNSET:
            geode->setColor(osg::Vec4d(1, 1, 0, 1));
            break;            
        default:
            LOG_WARN_S << "Unknown node type!";
            geode->setColor(osg::Vec4d(0,0,1,1));
    }
    
    if(fabs(node->getHeight()) > 30000)
    {
        //FIXME if nodes are too far aways, the culling mechanism of osg breaks.
        // I.e. verticves disappear when zooming in on them.
        LOG_WARN_S << "TraversabilityMap3dVisualization:: Ignoring node with height above +-30000 "; 
        return;
    }
    
    if(std::isnan(node->getHeight()))
        throw std::runtime_error("FOOOOOOOO");
    
    geode->drawHorizontalPlane(node->getHeight());
    
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
    Eigen::Vector2f toPos = (to->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    Eigen::Vector2f fromPos = (from->getIndex().cast<float>() + Eigen::Vector2f(0.5, 0.5)).array() * map.getResolution().cast<float>().array();
    osg::Vec3 fromOsg(fromPos.x(), fromPos.y(), from->getHeight());
    osg::Vec3 toOsg(toPos.x(), toPos.y(), to->getHeight());
    
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
    // Apply local frame.
    setLocalFrame(map.getLocalFrame());

    // Draw map extents.
    visualizeMapExtents(map.calculateCellExtents(), map.getResolution());

    //clear old data
    localNode->removeChildren(0, localNode->getNumChildren());

    nodeGeode = new PatchesGeode(map.getResolution().x(), map.getResolution().y());
    linesNode = new osgviz::LinesNode(osg::Vec4(1, 1, 1, 1));
    localNode->addChild(nodeGeode);
    localNode->addChild(linesNode);
    
    PatchesGeode *geode = dynamic_cast<PatchesGeode *>(nodeGeode.get());
    geode->setColor(osg::Vec4d(1,0,0,1));
    geode->setShowPatchExtents(true);
    geode->setShowNormals(true);
    
    for(size_t y = 0; y < map.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < map.getNumCells().x(); x++)
        {
            const LevelList<TraversabilityNodeBase *> &l(map.at(x, y));
            addNodeList(l, localNode);
        }
    }
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
    show_connections = val;
    emit propertyChanged("show_connections");
    setDirty();
}





