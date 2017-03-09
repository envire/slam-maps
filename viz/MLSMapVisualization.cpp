#include <iostream>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include <base/TimeMark.hpp>

#include <vizkit3d/ColorConversionHelper.hpp>
#include <maps/grid/OccupancyGridMap.hpp>

#include "MLSMapVisualization.hpp"

#include "PatchesGeode.hpp"

#include "ExtentsRectangle.hpp"

using namespace vizkit3d;
using namespace ::maps::grid;

template <class T, int options>
osg::Vec3 Vec3( const Eigen::Matrix<T,3,1, options>& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

template <class T, int options>
osg::Quat Quat( const Eigen::Quaternion<T, options>& q )
{
    return osg::Quat( q.x(), q.y(), q.z(), q.w() );
}

namespace vizkit3d {
struct PatchVisualizer
{
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::SLOPE>& p)
    {
        float minZ, maxZ;
        p.getRange(minZ, maxZ);
        minZ -= 5e-4f;
        maxZ += 5e-4f;
        Eigen::Vector3f normal = p.getNormal();
        if(normal.allFinite())
        {
            geode.drawPlane(Eigen::Hyperplane<float, 3>(p.getNormal(), p.getCenter()), minZ, maxZ);
        }
        else
        {
            float height = (maxZ - minZ) + 1e-3f;
            geode.drawBox(maxZ, height, osg::Vec3(0.f,0.f,1.f));
        }
    }
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::PRECALCULATED>& p)
    {
        float minZ, maxZ;
        p.getRange(minZ, maxZ);
        minZ -= 5e-4f;
        maxZ += 5e-4f;
        geode.drawPlane(Eigen::Hyperplane<float, 3>(p.getNormal(), p.getCenter()), minZ, maxZ, 1e-4f);
    }
    static void visualize(vizkit3d::PatchesGeode& geode, const SurfacePatch<MLSConfig::KALMAN>& p)
    {
        if(p.isHorizontal())
            geode.drawHorizontalPlane(p.getMean(), p.getStandardDeviation());
        else
            geode.drawBox(p.getMean(), p.getHeight(), Vec3(p.getNormal()));
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
    virtual void visualizeNegativeInformation(vizkit3d::PatchesGeode& geode) const = 0;
    virtual void setLocalFrame(osg::Node* node) = 0;
    virtual void setLocalFrame(osg::PositionAttitudeTransform* local_frame) = 0;
};

template<enum MLSConfig::update_model Type>
struct DataHold : public MLSMapVisualization::Data
{
private:
    MLSMap<Type> mls;
    base::Transform3d local_transform;
public:
    DataHold(const MLSMap<Type> mls_) : mls(mls_), local_transform(mls.getLocalFrame().inverse())
    {
        // reset local frames since they are modelled separately in the OSG tree
        mls.getLocalFrame() = base::Transform3d::Identity();
        boost::shared_ptr<maps::grid::OccupancyGridMap> grid;
        if(mls.hasFreeSpaceMap() && (grid = boost::dynamic_pointer_cast<maps::grid::OccupancyGridMap>(mls.getFreeSpaceMap())))
            grid->getLocalFrame() = base::Transform3d::Identity();
    }

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

        Eigen::Vector3d min_d = Eigen::Vector3d(min.x(), min.y(), 0.);
        Eigen::Vector3d max_d = Eigen::Vector3d(max.x(), max.y(), 0.);

        group->addChild( 
            new ExtentsRectangle( Eigen::Vector2d(min_d.x(), min_d.y()),
                Eigen::Vector2d(max_d.x(), max_d.y())));
    };

    void visualizeNegativeInformation(vizkit3d::PatchesGeode& geode) const
    {
        boost::shared_ptr<maps::grid::OccupancyGridMap> grid;
        if(mls.hasFreeSpaceMap() && (grid = boost::dynamic_pointer_cast<maps::grid::OccupancyGridMap>(mls.getFreeSpaceMap())))
        {
            Eigen::Vector3d res = grid->getVoxelResolution();
            maps::grid::Vector2ui num_cell = grid->getNumCells();
            const maps::grid::OccupancyConfiguration& config = grid->getConfig();
            for (size_t x = 0; x < num_cell.x(); x++)
            {
                for (size_t y = 0; y < num_cell.y(); y++)
                {
                    const maps::grid::OccupancyGridMap::GridMapBase::CellType &tree = grid->at(x, y);
                    maps::grid::Vector3d pos;
                    if(grid->fromGrid(maps::grid::Index(x,y), pos))
                    {
                        geode.setPosition(pos.x(), pos.y());
                        std::vector< std::pair<int,int> > free_cells;
                        for(maps::grid::DiscreteTree<maps::grid::OccupancyGridMap::VoxelCellType>::const_iterator cell_it = tree.begin(); cell_it != tree.end(); cell_it++)
                        {
                            if(cell_it->second.getLogOdds() < config.free_space_logodds)
                            {
                                if(free_cells.empty() || free_cells.back().second + 1 != cell_it->first)
                                    free_cells.push_back(std::make_pair(cell_it->first, cell_it->first));
                                else
                                    free_cells.back().second++;
                            }
                        }

                        for(const std::pair<int,int>& cell : free_cells)
                            geode.drawBox(tree.getCellCenter(cell.second) + res.z() * 0.5, res.z() * (float)((cell.second-cell.first)+1), osg::Vec3(0.f,0.f,1.f));
                    }
                }
            }
        }
    }

    void setLocalFrame(osg::Node* node)
    {
        osg::Transform* transform = node->asTransform();
        if(transform)
        {
            osg::PositionAttitudeTransform* local_frame = transform->asPositionAttitudeTransform();
            if(local_frame)
            {
                setLocalFrame(local_frame);
                return;
            }
        }
        std::cerr << "Couldn't set local transformation. The given osg::Node is not a osg::PositionAttitudeTransform" << std::endl;
    }

    void setLocalFrame(osg::PositionAttitudeTransform* local_frame)
    {
        local_frame->setPosition(Vec3(Eigen::Vector3d(local_transform.translation())));
        local_frame->setAttitude(Quat(Eigen::Quaterniond(local_transform.linear())));
    }
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
    showNormals(false),
    cycleHeightColor(true),
    cycleColorInterval(1.0),
    showPatchExtents(false),
    uncertaintyScale(1.0)
{
}

MLSMapVisualization::~MLSMapVisualization()
{
}

osg::ref_ptr<osg::Node> MLSMapVisualization::createMainNode()
{
    osg::ref_ptr<osg::PositionAttitudeTransform> local_frame(new osg::PositionAttitudeTransform);
    return local_frame;
}

void MLSMapVisualization::updateMainNode ( osg::Node* node )
{
    if(!p) return;
    p->setLocalFrame(node);

    osg::Group* group = node->asGroup();
    group->removeChildren(0, group->getNumChildren());

    Eigen::Vector2d res = p->getResolution();

    osg::ref_ptr<PatchesGeode> geode = new PatchesGeode(res.x(), res.y());
    group->addChild( geode );

    // draw the extents of the mls
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
    geode->setUncertaintyScale(uncertaintyScale);

    p->visualize(*geode);

    if( showUncertainty || showNormals || showPatchExtents)
    {
        geode->drawLines();
    }

    if( showNegative )
    {
        osg::ref_ptr<PatchesGeode> neg_geode = new PatchesGeode(res.x(), res.y());
        neg_geode->setColor(negativeCellColor);
        group->addChild( neg_geode );
        p->visualizeNegativeInformation(*neg_geode);
    }
}

void MLSMapVisualization::updateDataIntern(::maps::grid::MLSMapKalman const& value)
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
    if(enabled && (arePatchExtentsShown() || areNormalsShown()))
    {
        setShowPatchExtents(false);
        setShowNormals(false);
    }
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
    if(enabled && (arePatchExtentsShown() || isUncertaintyShown()))
    {
        setShowPatchExtents(false);
        setShowUncertainty(false);
    }
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
    if(value && (areNormalsShown() || isUncertaintyShown()))
    {
        setShowNormals(false);
        setShowUncertainty(false);
    }
    emit propertyChanged("show_patch_extents");
    setDirty();
}

bool MLSMapVisualization::arePatchExtentsShown() const
{
    return showPatchExtents;
}

double MLSMapVisualization::getUncertaintyScale() const
{
    return uncertaintyScale;
}

void MLSMapVisualization::setUncertaintyScale(double scaling)
{
    uncertaintyScale = std::abs(scaling);

    emit propertyChanged("uncertainty_scale");
    setDirty();
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(MLSMapVisualization)
