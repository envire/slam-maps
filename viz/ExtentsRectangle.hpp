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