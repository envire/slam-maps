#include <iostream>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include <base/TimeMark.hpp>

#include <vizkit3d/ColorConversionHelper.hpp>

#include "MLSMapVisualization.hpp"

#include "PatchesGeode.hpp"

#include "ExtentsRectangle.hpp"

using namespace vizkit3d;
using namespace ::maps::grid;

template <class T>
osg::Vec3 Vec3( const Eigen::Matrix<T,3,1>& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

namespace vizkit3d {
struct PatchVisualizer
{
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::PRECALCULATED>& p)
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
}

struct MLSMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    virtual ~Data() { }
    virtual Eigen::Vector2d getResolution() const = 0;
    virtual void visualize(vizkit3d::PatchesGeode& geode) const = 0;
    virtual void visualizeExtents(osg::Group* group) const = 0;
};

template<enum MLSConfig::update_model Type>
struct DataHold : public MLSMapVisualization::Data
{
    MLSMap<Type> mls;

    DataHold(const MLSMap<Type> mls_) : mls(mls_) {}

    Eigen::Vector2d getResolution() const { return mls.getResolution(); }
    void visualize(vizkit3d::PatchesGeode& geode) const
    {
        //const GridMap<SPListST> &mls = *this;
        Vector2ui num_cell = mls.getNumCells();
        for (size_t x = 0; x < num_cell.x(); x++)
        {
            for (size_t y = 0; y < num_cell.y(); y++)
            {
                typedef typename MLSMap<Type>::CellType Cell;
                const Cell &list = mls.at(x, y);

                Vector3d pos(0.00, 0.00, 0.00);
                mls.fromGrid(Index(x,y), pos);
                geode.setPosition(pos.x(), pos.y());
                for (typename Cell::const_iterator it = list.begin(); it != list.end(); it++)
                {
                    PatchVisualizer::visualize(geode, *it);
                } // for(SPList ...)
            } // for(y ...)
        } // for(x ...)        
    };
    
    void visualizeExtents(osg::Group* group) const
    {
        // get the color as a function of the group
        float scale = ((long)group%1000)/1000.0;
        osg::Vec4 col(0,0,0,1);
        vizkit3d::hslToRgb( scale, 1.0, 0.6, col.x(), col.y(), col.z() );
      
        CellExtents extents = mls.calculateCellExtents();

        Eigen::Vector2d min = extents.min().cast<double>().cwiseProduct(mls.getResolution());
        Eigen::Vector2d max = extents.max().cast<double>().cwiseProduct(mls.getResolution());

        Eigen::Vector3d min_d = mls.getLocalFrame().inverse() * Eigen::Vector3d(min.x(), min.y(), 0.);
        Eigen::Vector3d max_d = mls.getLocalFrame().inverse() * Eigen::Vector3d(max.x(), max.y(), 0.);

        group->addChild( 
            new ExtentsRectangle( Eigen::Vector2d(min_d.x(), min_d.y()),
                Eigen::Vector2d(max_d.x(), max_d.y())));
    };
};

MLSMapVisualization::MLSMapVisualization()
    : p(0),
    horizontalCellColor(osg::Vec4(0.1,0.5,0.9,1.0)),
    verticalCellColor(osg::Vec4(0.8,0.9,0.5,1.0)), 
    negativeCellColor(osg::Vec4(0.1,0.5,0.9,0.2)), 
    uncertaintyColor(osg::Vec4(0.5,0.1,0.1,0.3)), 
    showMapExtents(false),
    showUncertainty(false),
    showNegative(false),
    estimateNormals(false),
    cycleHeightColor(false),
    cycleColorInterval(1.0),
    showPatchExtents(true)
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

    // draw the extents of the mls
    group->removeChild( 1 );
    if( showMapExtents )
    {
        p->visualizeExtents(group);
    }

    if(cycleHeightColor)
    {
        geode->showCycleColor(true);
        geode->setCycleColorInterval(cycleColorInterval);
        geode->setColorHSVA(0, 1.0, 0.6, 1.0);
    }
    else
        geode->setColor(horizontalCellColor);
    geode->setShowPatchExtents(showPatchExtents);
    geode->setShowNormals(showNormals);

    base::TimeMark timer("MLS_VIZ::updateMainNode");
    p->visualize(*geode);

    if( showUncertainty || showNormals || showPatchExtents)
    {
        geode->drawLines();
    }

//    const double xo = mls.localFrameX();
//    const double yo = mls.localFrameY();

    std::cout << timer << std::endl;

}

void MLSMapVisualization::updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::KALMAN> const& value)
{
    p.reset(new DataHold<MLSConfig::KALMAN>( value ));
}
void MLSMapVisualization::updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::SLOPE> const& value)
{
    p.reset(new DataHold<MLSConfig::SLOPE>( value ));
}
void MLSMapVisualization::updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::PRECALCULATED> const& value)
{
    p.reset(new DataHold<MLSConfig::PRECALCULATED>( value ));
}

void MLSMapVisualization::setShowMapExtents(bool value)
{
    showMapExtents = value;
    emit propertyChanged("show_map_extents");
    setDirty();
}

bool MLSMapVisualization::areMapExtentsShown() const
{
    return showMapExtents;
}


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

void MLSMapVisualization::setShowPatchExtents( bool value ) 
{
    showPatchExtents = value;
    emit propertyChanged("show_patch_extents");
    setDirty();
}

bool MLSMapVisualization::arePatchExtentsShown() const
{
    return showPatchExtents;
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MLSMapVisualization)
