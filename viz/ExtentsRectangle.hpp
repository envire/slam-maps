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
#ifndef EXTENTSRECTANGLE_H
#define EXTENTSRECTANGLE_H

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>


class ExtentsRectangle : public osg::Geode
{
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec4Array> color;
    osg::ref_ptr<osg::Vec3Array> vertices;

public:
    ExtentsRectangle( Eigen::Vector2d min, Eigen::Vector2d max, 
        const osg::Vec4& col = osg::Vec4( 0.0f, 0.9f, 0.1f, 0.8f ) ) 
        : geom( new osg::Geometry() ),
          color( new osg::Vec4Array() ), 
          vertices( new osg::Vec3Array() )
    {
        vertices->push_back( osg::Vec3( min.x(), min.y(), 0 ));
        vertices->push_back( osg::Vec3( min.x(), max.y(), 0 ));
        vertices->push_back( osg::Vec3( max.x(), max.y(), 0 ));
        vertices->push_back( osg::Vec3( max.x(), min.y(), 0 ));

        geom->setVertexArray(vertices);
        osg::ref_ptr<osg::DrawArrays> drawArrays = 
            new osg::DrawArrays( osg::PrimitiveSet::LINE_LOOP, 0, vertices->size() );
        geom->addPrimitiveSet(drawArrays.get());

        color->push_back( col );
        geom->setColorArray(color.get());
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        addDrawable(geom.get());    

        osg::StateSet* ss = getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
        ss->setAttribute( new osg::LineWidth( 3.0 ) );
    }
};

#endif // EXTENTSRECTANGLE_H
