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
}