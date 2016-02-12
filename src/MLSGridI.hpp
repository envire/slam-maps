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
//    virtual void visualize(vizkit3d::PatchesGeode& geode) const = 0;

    virtual MLSBase* clone() const = 0;
//    virtual const Grid& getGrid() const = 0;
    virtual Grid& getGrid() = 0;

    struct MLSGridI; // will be templated
};


// TODO Make this and SPList templated
//    template<class SurfaceType>
struct MLSGrid::MLSBase::MLSGridI : public MLSGrid::MLSBase
{
    //typedef List<SurfaceType> SPList;
    GridMap<SPList> grid;

    MLSGridI(
            const Vector2ui &num_cells,
            const Vector2d &resolution,
            const MLSConfig &config_)
    : MLSBase(config_)
    , grid(num_cells, resolution, SPList(config_))
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

    Grid& getGrid()
    {
        return grid;
    }

    void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty);
};



}  // namespace maps

}  // namespace envire






#endif /* SRC_MLSGRIDI_HPP_ */
