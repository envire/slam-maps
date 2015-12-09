#include "PatchesGeode.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

namespace vizkit3d
{
    PatchesGeode::PatchesGeode()
        : vertex_index(0),
          hue(0.0),
          sat(1.0),
          alpha(1.0),
          lum(1.0),
          cycle_color(false)
    {
        colors = new osg::Vec4Array;
        vertices = new osg::Vec3Array;
        normals = new osg::Vec3Array;
        geom = new osg::Geometry;

        geom->setVertexArray(vertices);
        geom->setNormalArray(normals);
        geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        addDrawable(geom);
    }
    void PatchesGeode::drawPlane(
            const osg::Vec3& position,
            const osg::Vec3& extents,
            const osg::Vec3& mean,
            const osg::Vec3& normal)
    {
        // Sanity check (TODO should be disabled in release mode)
        for(int i=0; i<3; ++i)
            if(! (0.0f <= mean[i] && mean[i] <= extents[i]) )
            {
                std::cerr << "Mean of SurfacePatch is outside of its extents! {";
                for(int j=0; j<3; ++j) std::cerr << mean[j] << (j==2? "}   {" : ", ");
                for(int j=0; j<3; ++j) std::cerr << extents[j] << (j==2? "}\n" : ", ");
            }
        // first calculate the intersections of the plane with the borders of the box
        // Here, `extents` and `mean` are relative to the origin of the box
        float dist = mean*normal; // scalar product gives the signed distance from the origin

        // find the max coefficient of the normal:
        int i=0, j=1, k=2;
        if(std::abs(normal[i]) < std::abs(normal[j])) std::swap(i,j);
        if(std::abs(normal[i]) < std::abs(normal[k])) std::swap(i,k);

        osg::Vec3 prev_p;
        enum { NONE, LOW, BOX, HIGH } prev_pos = NONE, pos;
        // calculate intersections in direction k:
        for(int n=0; n<5; ++n)
        {
            osg::Vec3 p(0,0,0);
            float dotp = 0.0f;
            switch(n)
            {
            case 0: case 4: break;
            case 1: p[j] = extents[j]; dotp += extents[j] * normal[j]; break;
            case 2: p[j] = extents[j]; dotp += extents[j] * normal[j];
                // fall through
            case 3: p[k] = extents[k]; dotp += extents[k] * normal[k]; break;
            }
            p[i] = (dist - dotp) / normal[i];

            if( p[i] < 0.0f )
                pos = LOW;
            else if( p[i] > extents[i] )
                pos = HIGH;
            else
                pos = BOX;

            if( (prev_pos == LOW || prev_pos == HIGH) && pos != prev_pos )
            {
                // clipping in
                float h = prev_pos == LOW ? 0 : extents[i];
                float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
                osg::Vec3 cp = prev_p + (p - prev_p) * s;
                addVertex( position + cp, normal );
            }
            if( pos == BOX )
            {
                addVertex( position + p, normal );
            }
            else if( pos != prev_pos && prev_pos != NONE )
            {
                // clipping out
                float h = pos == LOW ? 0 : extents[i];
                float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
                osg::Vec3 cp = prev_p + (p - prev_p) * s;
                addVertex( position + cp, normal );
            }

            prev_pos = pos;
            prev_p = p;
        }

        closePolygon();
    }

    void PatchesGeode::drawPlane(
            const osg::Vec3& position,
            const osg::Vec4& heights,
            const osg::Vec3& extents,
            const osg::Vec3& normal,
            double min,
            double max )
    {
        const double xp = position.x();
        const double yp = position.y();
        const double zp = position.z();

        const double xs = extents.x();
        const double ys = extents.y();

        enum { NONE, LOW, BOX, HIGH } prev_pos = NONE, pos;
        osg::Vec3 prev_p, p;
        for( size_t i=0; i<4; i++ )
        {
            switch( i%4 )
            {
            case 0: p = osg::Vec3(xp-xs*0.5, yp-ys*0.5, heights[0]); break;
            case 1: p = osg::Vec3(xp+xs*0.5, yp-ys*0.5, heights[1]); break;
            case 2: p = osg::Vec3(xp+xs*0.5, yp+ys*0.5, heights[2]); break;
            case 3: p = osg::Vec3(xp-xs*0.5, yp+ys*0.5, heights[3]); break;
            }

            if( p.z() < min )
                pos = LOW;
            else if( p.z() > max )
                pos = HIGH;
            else
                pos = BOX;

            if( (prev_pos == LOW || prev_pos == HIGH) && pos != prev_pos )
            {
                // clipping in
                double h = prev_pos == LOW ? min : max;
                double s = (h - prev_p.z()) / (p.z() - prev_p.z());
                osg::Vec3 cp = prev_p + (p - prev_p) * s;
                addVertex( cp, normal );
            }
            if( pos == BOX )
            {
                addVertex( p, normal );
            }
            else if( pos != prev_pos && prev_pos != NONE )
            {
                // clipping out
                double h = pos == LOW ? min : max;
                double s = (h - prev_p.z()) / (p.z() - prev_p.z());
                osg::Vec3 cp = prev_p + (p - prev_p) * s;
                addVertex( cp, normal );
            }

            prev_pos = pos;
            prev_p = p;
        }

        closePolygon();
    }

    void PatchesGeode::drawBox(
            const osg::Vec3& position, 
            const osg::Vec3& extents, 
            const osg::Vec3& c_normal )
    {
        const double xp = position.x();
        const double yp = position.y();
        const double zp = position.z();

        const double xs = extents.x();
        const double ys = extents.y();
        const double zs = extents.z();

        const osg::Vec4 h( osg::Vec4(zp,zp,zp,zp) );
        osg::Vec3 normal( c_normal );

        addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]+zs*0.5), normal);
        addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]+zs*0.5), normal);
        addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]+zs*0.5), normal);
        addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]+zs*0.5), normal);

        if( zs > 0.0 )
        {
            normal = osg::Vec3(0,-1.0,0);
            addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]+zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]+zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[2]-zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[3]-zs*0.5), normal);

            normal = osg::Vec3(1.0,0,0);
            addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[0]+zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[1]+zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]-zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[3]-zs*0.5), normal);

            normal = osg::Vec3(0,1.0,0);
            addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[0]+zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[1]+zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[2]-zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[3]-zs*0.5), normal);

            normal = osg::Vec3(-1.0,0,0);
            addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[0]+zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[1]+zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[2]-zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]-zs*0.5), normal);

            normal = osg::Vec3(0,0,-1.0);
            addVertex(osg::Vec3(xp-xs*0.5, yp-ys*0.5, h[0]-zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp-ys*0.5, h[1]-zs*0.5), normal);
            addVertex(osg::Vec3(xp+xs*0.5, yp+ys*0.5, h[2]-zs*0.5), normal);
            addVertex(osg::Vec3(xp-xs*0.5, yp+ys*0.5, h[3]-zs*0.5), normal);
        }

        closeQuads();
    }

    void PatchesGeode::addVertex(const osg::Vec3& p, const osg::Vec3& n)
    {
        vertices->push_back( p );
        normals->push_back( n );

        if( cycle_color )
        {
            hue = (p.z() - std::floor(p.z() / cycle_color_interval) * cycle_color_interval) / cycle_color_interval;
            updateColor();
        }

        colors->push_back( color );
    }

    void PatchesGeode::updateColor()
    {
        vizkit3d::hslToRgb(hue, sat, lum , color.x(), color.y(), color.z());
        color.w() = alpha;
    }	

    void PatchesGeode::closePolygon()
    {
        geom->addPrimitiveSet(
                new osg::DrawArrays(
                        osg::PrimitiveSet::POLYGON,
                        vertex_index,
                        vertices->size() - vertex_index
                )
        );

        vertex_index = vertices->size();
    }

    void PatchesGeode::closeQuads()
    {
        geom->addPrimitiveSet(
                new osg::DrawArrays(
                        osg::PrimitiveSet::QUADS,
                        vertex_index,
                        vertices->size() - vertex_index
                )
        );

        vertex_index = vertices->size();
    }    

    void PatchesGeode::setColor(const osg::Vec4& color)
    {
        // TODO ideally this should also change the HSVA values
        this->color = color;
    }

    void PatchesGeode::setColorHSVA(float hue, float sat, float lum, float alpha)
    {
        this->hue = hue;
        this->sat = sat;
        this->alpha = alpha;
        this->lum = lum;

        updateColor();
    }

    void PatchesGeode::setCycleColorInterval(float cycle_color_interval)
    {
        this->cycle_color_interval = cycle_color_interval;
    }

    void PatchesGeode::showCycleColor(bool cycle_color)
    {
        this->cycle_color = cycle_color;
    }
} // namespace vizkit3d
