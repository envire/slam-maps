
#include "ContourMapVisualization.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

/** OSG **/
#include <osg/Geode>
#include <osg/LineWidth>

#include <osg/ShapeDrawable>

/** Std library **/
#include <iostream>

using namespace vizkit3d;

ContourMapVisualization::ContourMapVisualization()
{
    geom = new osg::Geometry;
    line_points = new osg::Vec3Array;
}

ContourMapVisualization::~ContourMapVisualization()
{
}

osg::ref_ptr<osg::Node> ContourMapVisualization::createMainNode()
{
    /** Set the osg data **/
    geom->setVertexArray(line_points);
    draw_arrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, line_points->size() );
    geom->addPrimitiveSet(draw_arrays.get());

    /**  Add the Geometry (Drawable) to a Geode **/
    geode = new osg::Geode;
    geode->addDrawable(geom.get());

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    return geode;
}

void ContourMapVisualization::updateMainNode( osg::Node* node )
{
    line_points->clear();

    /** From the internal storage to the drawable OSG data **/
    for (maps::ContourMap::const_iterator it = this->contour_lines.begin();
        it != this->contour_lines.end(); ++it)
    {
        maps::Point3d const &psi_a = (*it).psi_a();
        line_points->push_back(osg::Vec3(psi_a.x(), psi_a.y(), psi_a.z()));

        maps::Point3d const &psi_b = (*it).psi_b();
        line_points->push_back(osg::Vec3(psi_b.x(), psi_b.y(), psi_b.z()));
    }

    geom->setVertexArray(line_points);
    draw_arrays->setCount(line_points->size());
}

void ContourMapVisualization::updateDataIntern(const ::maps::ContourMap & data)
{
    this->contour_lines = data;
}

