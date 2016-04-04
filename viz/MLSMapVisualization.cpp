#include <iostream>

#include "MLSMapVisualization.hpp"

#include "PatchesGeode.hpp"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include <base/TimeMark.hpp>

#include <maps/grid/MLSMapI.hpp>

using namespace vizkit3d;
using namespace ::maps;

template <class T>
osg::Vec3 Vec3( const Eigen::Matrix<T,3,1>& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}


struct MLSMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    virtual ~Data() { }
    virtual Eigen::Vector2d getResolution() const = 0;
    virtual void visualize(vizkit3d::PatchesGeode& geode) const = 0;

};

template<enum MLSConfig::update_model Type>
struct DataHold : public MLSMapVisualization::Data
{
    MLSMapI<Type> mls;
    DataHold(const MLSMapI<Type> mls_) : mls(mls_) {}
    Eigen::Vector2d getResolution() const { return mls.getResolution(); }
    void visualize(vizkit3d::PatchesGeode& geode) const;
};


MLSMapVisualization::MLSMapVisualization()
    : p(0),
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

MLSMapVisualization::~MLSMapVisualization()
{
}

osg::ref_ptr<osg::Node> MLSMapVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild(geode.get());

    return group.release();
}

void MLSMapVisualization::updateMainNode ( osg::Node* node )
{
    if(!p) return;
    osg::Group* group = static_cast<osg::Group*>(node);    

//    MLSMap &mls = p->data;
    Eigen::Vector2d res = p->getResolution();

    osg::ref_ptr<PatchesGeode> geode = new PatchesGeode(res.x(), res.y());
    group->setChild( 0, geode );


    if(cycleHeightColor)
    {
        geode->showCycleColor(true);
        geode->setCycleColorInterval(cycleColorInterval);
        geode->setColorHSVA(0, 1.0, 0.6, 1.0);
    }
    else
        geode->setColor(horizontalCellColor);
    geode->setShowExtents(showExtents);
    geode->setShowNormals(showNormals);

    base::TimeMark timer("MLS_VIZ::updateMainNode");
    p->visualize(*geode);

    if( showUncertainty || showNormals || showExtents)
    {
        geode->drawLines();
    }

//    const double xo = mls.localFrameX();
//    const double yo = mls.localFrameY();

    std::cout << timer << std::endl;

}
#if 0
void MLSMap::visualize(vizkit3d::PatchesGeode& geode) const
{

    switch(map->mls_config.updateModel)
    {
    case MLSConfig::SLOPE:
        dynamic_cast<const MLSMapI<SurfacePatchT<MLSConfig::SLOPE> >&>(*map).visualize(geode);
        break;
    case MLSConfig::KALMAN:
        dynamic_cast<const MLSMapI<SurfacePatchT<MLSConfig::KALMAN> >&>(*map).visualize(geode);
        break;
    default:
        throw std::runtime_error("Can't visualize unknown map type");
    }
}
#endif
namespace maps {


struct PatchVisualizer
{
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::SLOPE>& p)
    {
        if( !p.isNegative() )
        {
            float minZ, maxZ;
            p.getRange(minZ, maxZ);
//                        float stdev = p.getStdev() + 1e-4f;
            float zp= (maxZ+minZ)*0.5f;
            float height = (maxZ - minZ) + 1e-3f;
            osg::Vec3 mean = Vec3(p.getCenter());
            mean.z() -= zp;
            osg::Vec3 normal = Vec3(p.getNormal());
            geode.drawPlane(zp, height, mean, normal);
        }
    }
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::KALMAN>& p)
    {
        {
            geode.drawBox(p.mean, p.height, Vec3(p.getNormal()));
        }
    }
};

} // namespace maps

template<enum MLSConfig::update_model Type>
void DataHold<Type>::visualize(vizkit3d::PatchesGeode& geode) const
{
    //const GridMap<SPListST> &mls = *this;
    Vector2ui num_cell = mls.getNumCells();
    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            typedef typename MLSMapI<Type>::SPListST SPListST;
            const SPListST &list = mls.at(x, y);

            Vector3d pos(0.00, 0.00, 0.00);
            mls.fromGrid(Index(x,y), pos);
            geode.setPosition(pos.x(), pos.y());
            for (typename SPListST::const_iterator it = list.begin(); it != list.end(); it++)
            {
                PatchVisualizer::visualize(geode, *it);
            } // for(SPList ...)
        } // for(y ...)
    } // for(x ...)




}  // namespace maps


void MLSMapVisualization::updateDataIntern(::maps::MLSMapKalman const& value)
{
    p.reset(new DataHold<MLSConfig::KALMAN>( value ));
}
void MLSMapVisualization::updateDataIntern(::maps::MLSMapSloped const& value)
{
    p.reset(new DataHold<MLSConfig::SLOPE>( value ));
}

#if 0
osg::Vec3 MLSMapVisualization::estimateNormal(const MLSMap &grid, const SurfacePatch &patch, const Index &patch_idx) const
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
#endif


bool MLSMapVisualization::isUncertaintyShown() const
{
    return showUncertainty;
}

void MLSMapVisualization::setShowUncertainty(bool enabled)
{
    showUncertainty = enabled;
    emit propertyChanged("show_uncertainty");
    setDirty();
}

bool MLSMapVisualization::isNegativeShown() const
{
    return showNegative;
}

void MLSMapVisualization::setShowNegative(bool enabled)
{
    showNegative = enabled;
    emit propertyChanged("show_negative");
    setDirty();
}

bool MLSMapVisualization::areNormalsEstimated() const
{
    return estimateNormals;
}

void MLSMapVisualization::setEstimateNormals(bool enabled)
{
    estimateNormals = enabled;
    emit propertyChanged("estimate_normals");
    setDirty();
}

bool MLSMapVisualization::areNormalsShown() const
{
    return showNormals;
}

void MLSMapVisualization::setShowNormals(bool enabled)
{
    showNormals = enabled;
    emit propertyChanged("show_normals");
    setDirty();
}

bool MLSMapVisualization::isHeightColorCycled() const
{
    return cycleHeightColor;
}

void MLSMapVisualization::setCycleHeightColor(bool enabled)
{
    cycleHeightColor = enabled;
    emit propertyChanged("cycle_height_color");
    setDirty();
}

double MLSMapVisualization::getCycleColorInterval() const
{
    return cycleColorInterval;
}

void MLSMapVisualization::setCycleColorInterval(double interval)
{
    if(interval == 0.0)
        cycleColorInterval = 1.0;
    else
        cycleColorInterval = interval;
    emit propertyChanged("cycle_color_interval");
    setDirty();
}

QColor MLSMapVisualization::getHorizontalCellColor() const
{
    QColor color;
    color.setRgbF(horizontalCellColor.x(), horizontalCellColor.y(), horizontalCellColor.z(), horizontalCellColor.w());
    return color;
}

void MLSMapVisualization::setHorizontalCellColor(QColor color)
{
    horizontalCellColor.x() = color.redF();
    horizontalCellColor.y() = color.greenF();
    horizontalCellColor.z() = color.blueF();
    horizontalCellColor.w() = color.alphaF();
    emit propertyChanged("horizontal_cell_color");
    setDirty();
}

QColor MLSMapVisualization::getVerticalCellColor() const
{
    QColor color;
    color.setRgbF(verticalCellColor.x(), verticalCellColor.y(), verticalCellColor.z(), verticalCellColor.w());
    return color;
}

void MLSMapVisualization::setVerticalCellColor(QColor color)
{
    verticalCellColor.x() = color.redF();
    verticalCellColor.y() = color.greenF();
    verticalCellColor.z() = color.blueF();
    verticalCellColor.w() = color.alphaF();
    emit propertyChanged("vertical_cell_color");
    setDirty();
}

QColor MLSMapVisualization::getNegativeCellColor() const
{
    QColor color;
    color.setRgbF(negativeCellColor.x(), negativeCellColor.y(), negativeCellColor.z(), negativeCellColor.w());
    return color;
}

void MLSMapVisualization::setNegativeCellColor(QColor color)
{
    negativeCellColor.x() = color.redF();
    negativeCellColor.y() = color.greenF();
    negativeCellColor.z() = color.blueF();
    negativeCellColor.w() = color.alphaF();
    emit propertyChanged("negative_cell_color");
    setDirty();
}

QColor MLSMapVisualization::getUncertaintyColor() const
{
    QColor color;
    color.setRgbF(uncertaintyColor.x(), uncertaintyColor.y(), uncertaintyColor.z(), uncertaintyColor.w());
    return color;
}

void MLSMapVisualization::setUncertaintyColor(QColor color)
{
    uncertaintyColor.x() = color.redF();
    uncertaintyColor.y() = color.greenF();
    uncertaintyColor.z() = color.blueF();
    uncertaintyColor.w() = color.alphaF();
    emit propertyChanged("uncertainty_color");
    setDirty();
}

void MLSMapVisualization::setShowExtents( bool value ) 
{
    showExtents = value;
    emit propertyChanged("show_extents");
    setDirty();
}

bool MLSMapVisualization::areExtentsShown() const
{
    return showExtents;
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MLSMapVisualization)
