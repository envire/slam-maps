#include <iostream>

#include "MLSGridVisualization.hpp"

#include "PatchesGeode.hpp"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include <base/TimeMark.hpp>

using namespace vizkit3d;
using namespace envire::maps;

template <class T>
osg::Vec3 Vec3( const Eigen::Matrix<T,3,1>& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

struct MLSGridVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    envire::maps::MLSGrid data;
};


MLSGridVisualization::MLSGridVisualization()
    : p(new Data),
    horizontalCellColor(osg::Vec4(0.1,0.5,0.9,1.0)), 
    verticalCellColor(osg::Vec4(0.8,0.9,0.5,1.0)), 
    negativeCellColor(osg::Vec4(0.1,0.5,0.9,0.2)), 
    uncertaintyColor(osg::Vec4(0.5,0.1,0.1,0.3)), 
    showUncertainty(false),
    showNegative(false),
    estimateNormals(false),
    cycleHeightColor(false),
    cycleColorInterval(1.0),
    showExtents(true)
{
}

MLSGridVisualization::~MLSGridVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> MLSGridVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild(geode.get());

    return group.release();
}

void MLSGridVisualization::updateMainNode ( osg::Node* node )
{
    osg::Group* group = static_cast<osg::Group*>(node);    

    osg::ref_ptr<PatchesGeode> geode = new PatchesGeode();
    group->setChild( 0, geode );

    MLSGrid &mls = p->data;

    /*// draw the extents of the mls
    group->removeChild( 1 );
    if( showExtents )
    {
        // get the color as a function of the environmentitem pointer
        float scale = ((long)item%1000)/1000.0;
        osg::Vec4 col(0,0,0,1);
        vizkit3d::hslToRgb( scale, 1.0, 0.6, col.x(), col.y(), col.z() );

        group->addChild( 
            new ExtentsRectangle( mls->getExtents(), col ) );
    }    */

    Eigen::Vector2d res = mls.getResolution();
    Vector2ui num_cell = mls.getNumCells();
    const double xs = res.x();
    const double ys = res.y();

//    const double xo = mls.localFrameX();
//    const double yo = mls.localFrameY();

    osg::ref_ptr<osg::Vec3Array> var_vertices = new osg::Vec3Array;
    base::TimeMark timer("MLS_VIZ::updateMainNode");

    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const SPList &list = mls.at(x, y);

            for (SPList::const_iterator it = list.begin(); it != list.end(); it++)
            {

                const SurfacePatch &p(*it);
                Vector3d pos;
                mls.fromGrid(Index(x,y), pos);
                double xp = pos.x();
                double yp = pos.y();

                // setup the color for the next geometry
                if(mls.getConfig().useColor == true)
                {
                    geode->showCycleColor(false);
                    base::Vector3d c = p.getColor();
                    osg::Vec4 col = osg::Vec4(c.x(), c.y(), c.z(), 1.0);
                    geode->setColor(col);
                }
                else if(cycleHeightColor)
                {
                    geode->showCycleColor(true);
                    geode->setCycleColorInterval(cycleColorInterval);
                    double hue = (p.getMean() - std::floor(p.getMean() / cycleColorInterval) * cycleColorInterval) / cycleColorInterval;
                    double sat = 1.0;
                    double lum = 0.6;
                    double alpha = std::max(0.0, 1.0 - p.getStdev());
                    geode->setColorHSVA(hue, sat, lum, alpha);
                }
                else
                    geode->setColor(horizontalCellColor);

                // slopes need to be handled differently
                if( mls.getConfig().updateModel == MLSConfig::SLOPE )
                {
                    if( !p.isNegative() )
                    {
                        float minZ, maxZ;
                        p.getRange(minZ, maxZ);
//                        float stdev = p.getStdev() + 1e-4f;
                        osg::Vec3 position(xp, yp, (maxZ+minZ)*0.5f);
                        osg::Vec3 extents(xs, ys, (maxZ - minZ) + 1e-3f);
                        osg::Vec3 mean = Vec3(p.getCenter());
                        mean.z() -= position.z();
                        osg::Vec3 normal = Vec3(p.getNormal());
                        geode->drawPlane(position, extents * 0.5f, mean, normal);
                        osg::Vec3 center = position + mean;

                        if(showNormals)
                        {
                            var_vertices->push_back(center);
                            var_vertices->push_back(center+normal*0.1);
                        }
                        if(showExtents)
                        {
                            var_vertices->push_back(osg::Vec3(xp, yp, minZ));
                            var_vertices->push_back(osg::Vec3(xp, yp, maxZ));
                        }
                    }
                    else if (showNegative)
                    {
                        geode->setColor( negativeCellColor );
                        geode->drawBox(
                                osg::Vec3( xp, yp, p.getMean()-p.getHeight()*.5 ),
                                osg::Vec3( xs, ys, p.getHeight() ),
                                osg::Vec3(0, 0, 1.0) );
                    }
                }
                else
                {
                    if( p.isHorizontal() )
                    {
                        geode->drawBox( 
                                osg::Vec3( xp, yp, p.getMean() ), 
                                osg::Vec3( xs, ys, 0.0 ), 
                                estimateNormals ? 
                                    estimateNormal(mls, p, Index(x,y)) :
                                    osg::Vec3( 0, 0, 1.0 ) );
                    }
                    else
                    {
                        if( p.isVertical() || showNegative )
                        {   
                            geode->setColor( 
                                    p.isVertical() ? verticalCellColor : negativeCellColor );
                            geode->drawBox( 
                                    osg::Vec3( xp, yp, p.getMean()-p.getHeight()*.5 ), 
                                    osg::Vec3( xs, ys, p.getHeight() ), 
                                    osg::Vec3(0, 0, 1.0) );
                        }
                    }
                }
            } // for(SPList ...)
        } // for(y ...)
    } // for(x ...)

    if( showUncertainty || showNormals || showExtents)
    {
        osg::ref_ptr<osg::Geometry> var_geom = new osg::Geometry;
        var_geom->setVertexArray( var_vertices );
        osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, var_vertices->size() );
        var_geom->addPrimitiveSet(drawArrays.get());

        osg::ref_ptr<osg::Vec4Array> var_color = new osg::Vec4Array;
        var_color->push_back( osg::Vec4( 0.5, 0.1, 0.8, 1.0 ) );
        var_geom->setColorArray( var_color.get() );
        var_geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        geode->addDrawable( var_geom.get() );
    }

    std::cout << timer << std::endl;
}


void MLSGridVisualization::updateDataIntern(envire::maps::MLSGrid const& value)
{
    p->data = value;
}

osg::Vec3 MLSGridVisualization::estimateNormal(const MLSGrid &grid, const SurfacePatch &patch, const Index &patch_idx) const
{
    Vector3d patch_pos;
    if(!grid.fromGrid(patch_idx, patch_pos))
        return osg::Vec3(0,0,0);

    Vector3d patch_center(patch_pos.x(), patch_pos.y(), patch.getMean());

    Vector3d d[2] = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };

    for( int n = 0; n < 2; n++ )
    {
        for( int i = -1; i < 2; i += 2 )
        {
            Index idx = patch_idx + Index(n * i, (n - 1) * i );
            if (grid.inGrid(idx))
            {
                // instead stddeviation of the patch use (grid.getScaleX() * 2)
                SPList::const_iterator it = grid.at(idx).getPatchByZ(patch.getMean(), grid.getResolution().sum());
                if( it != grid.at(idx).end() )
                {
                    Vector3d pos(-1, -1, 0);
                    grid.fromGrid(idx, pos);

                    Eigen::Vector3d v(pos.x(), pos.y(), it->getMean());
                    d[n] += (v - patch_center)*i;
                }
            }
        }
    }

    Eigen::Vector3d n = d[0].cross( d[1] );
    if( n.norm() > 0.0 )
    {
        n.normalize();
        return osg::Vec3(n.x(), n.y(), n.z());
    }
    else
        return osg::Vec3(0,0,1.0);
}

bool MLSGridVisualization::isUncertaintyShown() const
{
    return showUncertainty;
}

void MLSGridVisualization::setShowUncertainty(bool enabled)
{
    showUncertainty = enabled;
    emit propertyChanged("show_uncertainty");
    setDirty();
}

bool MLSGridVisualization::isNegativeShown() const
{
    return showNegative;
}

void MLSGridVisualization::setShowNegative(bool enabled)
{
    showNegative = enabled;
    emit propertyChanged("show_negative");
    setDirty();
}

bool MLSGridVisualization::areNormalsEstimated() const
{
    return estimateNormals;
}

void MLSGridVisualization::setEstimateNormals(bool enabled)
{
    estimateNormals = enabled;
    emit propertyChanged("estimate_normals");
    setDirty();
}

bool MLSGridVisualization::areNormalsShown() const
{
    return showNormals;
}

void MLSGridVisualization::setShowNormals(bool enabled)
{
    showNormals = enabled;
    emit propertyChanged("show_normals");
    setDirty();
}

bool MLSGridVisualization::isHeightColorCycled() const
{
    return cycleHeightColor;
}

void MLSGridVisualization::setCycleHeightColor(bool enabled)
{
    cycleHeightColor = enabled;
    emit propertyChanged("cycle_height_color");
    setDirty();
}

double MLSGridVisualization::getCycleColorInterval() const
{
    return cycleColorInterval;
}

void MLSGridVisualization::setCycleColorInterval(double interval)
{
    if(interval == 0.0)
        cycleColorInterval = 1.0;
    else
        cycleColorInterval = interval;
    emit propertyChanged("cycle_color_interval");
    setDirty();
}

QColor MLSGridVisualization::getHorizontalCellColor() const
{
    QColor color;
    color.setRgbF(horizontalCellColor.x(), horizontalCellColor.y(), horizontalCellColor.z(), horizontalCellColor.w());
    return color;
}

void MLSGridVisualization::setHorizontalCellColor(QColor color)
{
    horizontalCellColor.x() = color.redF();
    horizontalCellColor.y() = color.greenF();
    horizontalCellColor.z() = color.blueF();
    horizontalCellColor.w() = color.alphaF();
    emit propertyChanged("horizontal_cell_color");
    setDirty();
}

QColor MLSGridVisualization::getVerticalCellColor() const
{
    QColor color;
    color.setRgbF(verticalCellColor.x(), verticalCellColor.y(), verticalCellColor.z(), verticalCellColor.w());
    return color;
}

void MLSGridVisualization::setVerticalCellColor(QColor color)
{
    verticalCellColor.x() = color.redF();
    verticalCellColor.y() = color.greenF();
    verticalCellColor.z() = color.blueF();
    verticalCellColor.w() = color.alphaF();
    emit propertyChanged("vertical_cell_color");
    setDirty();
}

QColor MLSGridVisualization::getNegativeCellColor() const
{
    QColor color;
    color.setRgbF(negativeCellColor.x(), negativeCellColor.y(), negativeCellColor.z(), negativeCellColor.w());
    return color;
}

void MLSGridVisualization::setNegativeCellColor(QColor color)
{
    negativeCellColor.x() = color.redF();
    negativeCellColor.y() = color.greenF();
    negativeCellColor.z() = color.blueF();
    negativeCellColor.w() = color.alphaF();
    emit propertyChanged("negative_cell_color");
    setDirty();
}

QColor MLSGridVisualization::getUncertaintyColor() const
{
    QColor color;
    color.setRgbF(uncertaintyColor.x(), uncertaintyColor.y(), uncertaintyColor.z(), uncertaintyColor.w());
    return color;
}

void MLSGridVisualization::setUncertaintyColor(QColor color)
{
    uncertaintyColor.x() = color.redF();
    uncertaintyColor.y() = color.greenF();
    uncertaintyColor.z() = color.blueF();
    uncertaintyColor.w() = color.alphaF();
    emit propertyChanged("uncertainty_color");
    setDirty();
}

void MLSGridVisualization::setShowExtents( bool value ) 
{
    showExtents = value;
    emit propertyChanged("show_extents");
    setDirty();
}

bool MLSGridVisualization::areExtentsShown() const
{
    return showExtents;
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MLSGridVisualization)
