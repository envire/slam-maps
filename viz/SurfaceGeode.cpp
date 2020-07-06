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
#include "SurfaceGeode.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

#include <maps/tools/SurfaceIntersection.hpp>

namespace vizkit3d
{
    SurfaceGeode::SurfaceGeode(float x_res, float y_res)
        : vertex_index(0),
          xp(0), yp(0),
          xs(x_res), ys(y_res),
          hue(0.0),
          sat(1.0),
          alpha(1.0),
          lum(1.0),
          cycle_color(false),
          uncertaintyScale(1.0)
    {
        colors = new osg::Vec4Array;
        vertices = new osg::Vec3Array;
        normals = new osg::Vec3Array;
        geom = new osg::Geometry;
        var_vertices = new osg::Vec3Array;

        showNormals = true;
        showPatchExtents = false;

        geom->setUseVertexBufferObjects(true);

        geom->setVertexArray(vertices);
        geom->setNormalArray(normals);
        geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        addDrawable(geom);
    }

    void SurfaceGeode::addVertex(const osg::Vec3& p, const osg::Vec3& n, const float & stdev)
    {
        vertices->push_back( p );
        normals->push_back( n );

        if( cycle_color )
        {
            hue = (p.z() - std::floor(p.z() / cycle_color_interval) * cycle_color_interval) / cycle_color_interval;
            alpha = std::max( 0.0, (uncertaintyScale - stdev) / uncertaintyScale);
            updateColor();
        }

        colors->push_back( color );

    }


    void SurfaceGeode::updateColor()
    {
        vizkit3d::hslToRgb(hue, sat, lum , color.x(), color.y(), color.z());
        color.w() = alpha;
    }

    void SurfaceGeode::setColor(const osg::Vec4& color)
    {
        // TODO ideally this should also change the HSVA values
        this->color = color;
    }

    void SurfaceGeode::setColorHSVA(float hue, float sat, float lum, float alpha)
    {
        this->hue = hue;
        this->sat = sat;
        this->alpha = alpha;
        this->lum = lum;

        updateColor();
    }

    void SurfaceGeode::setCycleColorInterval(float cycle_color_interval)
    {
        this->cycle_color_interval = cycle_color_interval;
    }

    void SurfaceGeode::showCycleColor(bool cycle_color)
    {
        this->cycle_color = cycle_color;
    }

    void SurfaceGeode::setUncertaintyScale(double uncertainty_scale)
    {
        this->uncertaintyScale = uncertainty_scale;
    }

    void SurfaceGeode::drawLines()
    {
        osg::ref_ptr<osg::Geometry> var_geom = new osg::Geometry;
        var_geom->setVertexArray( var_vertices );
        osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, var_vertices->size() );
        var_geom->addPrimitiveSet(drawArrays.get());

        osg::ref_ptr<osg::Vec4Array> var_color = new osg::Vec4Array;
        var_color->push_back( osg::Vec4( 0.5, 0.1, 0.8, 1.0 ) );
        var_geom->setColorArray( var_color.get() );
        var_geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        addDrawable( var_geom.get() );
    }
} // namespace vizkit3d
