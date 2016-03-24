#ifndef __MAPS_MLSMAPI_HPP__
#define __MAPS_MLSMAPI_HPP__


#include "MLSMap.hpp"


namespace maps {

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


template<class SurfaceType>
struct MLSMapI : public MLSMap::MLSBase
{
    typedef SPList<SurfaceType> SPListST;
    GridMap<SPListST> grid;

    MLSMapI(
            const Vector2ui &num_cells,
            const Vector2d &resolution,
            const MLSConfig &config_)
    : MLSBase(config_)
    , grid(num_cells, resolution, SPListST(config_))
    {
        // assert that config is compatible to SurfaceType ...
    }
    MLSMapI() : MLSBase(MLSConfig())
    {
        // empty
    }

    void merge(const MLSBase& other)
    {
        // TODO
    }

    void visualize(vizkit3d::PatchesGeode& geode) const;

    MLSBase* clone() const
    {
        return new MLSMapI(*this);
    }

    base::Transform3d& getLocalFrame()
    {
        return grid.getLocalFrame();
    }
    Eigen::Vector2d getSize() const
    {
        return grid.getSize();
    }
    Eigen::Vector2d getResolution() const
    {
        return grid.getResolution();
    }

    void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty);
    void mergePoint(const Eigen::Vector3d& point);
};



}  // namespace maps

#endif /* __MAPS_MLSMAPI_HPP__ */
