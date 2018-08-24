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

#include "ContourMapVisualization.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

/** OSG **/
#include <osg/Geode>
#include <osg/LineWidth>

#include <osg/ShapeDrawable>

/** Std library **/
#include <iostream>

using namespace vizkit3d;

ContourMapVisualization::ContourMapVisualization() : MapVisualization< maps::geometric::ContourMap >()
{
    geom = new osg::Geometry;
    line_points = new osg::Vec3Array;
    color_array = new osg::Vec4Array;

}

ContourMapVisualization::~ContourMapVisualization()
{
}

osg::ref_ptr<osg::Node> ContourMapVisualization::createMainNode()
{
    /** Create main node. **/
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();

    /** Set the osg data **/
    geom->setVertexArray(line_points);
    draw_arrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, line_points->size() );
    geom->addPrimitiveSet(draw_arrays.get());

    /**  Add the Geometry (Drawable) to a Geode **/
    geode = new osg::Geode;
    geode->addDrawable(geom.get());

    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    /** Add geode to main node. **/
    mainNode->addChild(geode.get());

    return mainNode;
}

void ContourMapVisualization::updateMainNode( osg::Node* node )
{
    line_points->clear();

    /** From the internal storage to the drawable OSG data **/
    for (::maps::geometric::ContourMap::const_iterator it = this->contour_lines.begin();
        it != this->contour_lines.end(); ++it)
    {
        ::maps::geometric::Point3d const &psi_a = (*it).psi_a();
        line_points->push_back(osg::Vec3(psi_a.x(), psi_a.y(), psi_a.z()));

        ::maps::geometric::Point3d const &psi_b = (*it).psi_b();
        line_points->push_back(osg::Vec3(psi_b.x(), psi_b.y(), psi_b.z()));
    }

    geom->setVertexArray(line_points);
    draw_arrays->setCount(line_points->size());

    /** Apply local frame. **/
    setLocalFrame(contour_lines.getLocalFrame());
}

void ContourMapVisualization::updateDataIntern(const ::maps::geometric::ContourMap & data)
{
    this->contour_lines = data;
}

void ContourMapVisualization::setColor(QColor color)
{
    this->setColor( color.redF(), color.greenF(), color.blueF(), color.alphaF() );
    emit propertyChanged("Color");
}

void ContourMapVisualization::setColor(double r, double g, double b, double a)
{
    color = osg::Vec4( r, g, b, a );

    // set colors
    color_array->clear();
    color_array->push_back( color );
    geom->setColorArray( color_array );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
}


void ContourMapVisualization::setColor(const base::Vector3d& color)
{
    this->setColor(color.x(), color.y(), color.z(), 1.0);
    emit propertyChanged("Color");
}


QColor ContourMapVisualization::getColor() const
{
    QColor color;
    color.setRgbF(this->color.x(), this->color.y(), this->color.z(), this->color.w());
    return color;
}

double ContourMapVisualization::getLineWidth()
{
    return line_width;
}

void ContourMapVisualization::setLineWidth(double line_width)
{
    this->line_width = line_width;
    if(geode)
    {
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(line_width);
        stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    }
    emit propertyChanged("LineWidth");
}
