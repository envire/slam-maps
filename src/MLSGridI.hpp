#ifndef SRC_MLSGRIDI_HPP_
#define SRC_MLSGRIDI_HPP_


#include "MLSGrid.hpp"


namespace envire {

namespace maps {

class MLSGrid::MLSBase
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
struct MLSGridI : public MLSGrid::MLSBase
{
    typedef SPList<SurfaceType> SPListST;
    GridMap<SPListST> grid;

    MLSGridI(
            const Vector2ui &num_cells,
            const Vector2d &resolution,
            const MLSConfig &config_)
    : MLSBase(config_)
    , grid(num_cells, resolution, SPListST(config_))
    {
        // assert that config is compatible to SurfaceType ...
    }
    MLSGridI() : MLSBase(MLSConfig())
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
        return new MLSGridI(*this);
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

}  // namespace envire






#endif /* SRC_MLSGRIDI_HPP_ */
