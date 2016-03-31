#ifndef __MAPS_MLSMAPI_HPP__
#define __MAPS_MLSMAPI_HPP__


#include "MLSMap.hpp"

#include "MLGrid.hpp"


namespace maps {
#if 0
class MLSMap::MLSBase
{
public:
    MLSConfig mls_config;
    MLSBase(const MLSConfig &config_) : mls_config(config_) {}
    virtual ~MLSBase() { };

    virtual void merge(const MLSBase& other) = 0;
    virtual void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty) = 0;
    virtual void mergePoint(const Eigen::Vector3d& point) = 0;
//    virtual void visualize(vizkit3d::PatchesGeode& geode) const = 0;

    virtual MLSBase* clone() const = 0;
//    virtual const Grid& getGrid() const = 0;
    virtual Eigen::Vector2d getResolution() const= 0;
    virtual Eigen::Vector2d getSize() const= 0;
    virtual base::Transform3d& getLocalFrame() = 0;

};

#endif

template<enum MLSConfig::update_model  SurfaceType>
struct MLSMapI : public MLGrid<SurfacePatchT<SurfaceType> >
{
    typedef SurfacePatchT<SurfaceType> Patch;
    typedef LevelList<Patch>SPListST; // TODO rename
    typedef MLGrid<Patch> Base;
//    GridMap<SPListST> grid;

    MLSConfig config;

    MLSMapI(
            const Vector2ui &num_cells,
            const Vector2d &resolution,
            const MLSConfig &config_)
    : Base(num_cells, resolution)
    , config(config_)
    {
        // TODO assert that config is compatible to SurfaceType ...
    }
    MLSMapI()
    {
        // empty
    }

    void merge(const MLSMapI& other)
    {
        // TODO
    }

    void visualize(vizkit3d::PatchesGeode& geode) const;


    void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = false);
    void mergePoint(const Eigen::Vector3d& point);
};



}  // namespace maps

#endif /* __MAPS_MLSMAPI_HPP__ */
