#ifndef __MAPS_MLSMAPI_HPP__
#define __MAPS_MLSMAPI_HPP__


#include "MLSMap.hpp"

#include "MLGrid.hpp"


namespace maps { namespace grid
{

    template<enum MLSConfig::update_model  SurfaceType>
    struct MLSMapI : public MLGrid<SurfacePatch<SurfaceType> >
    {
        typedef SurfacePatch<SurfaceType> Patch;
        typedef LevelList<Patch>SPListST; // TODO rename
        typedef MLGrid<Patch> Base;

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

        void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = false);
        void mergePoint(const Eigen::Vector3d& point);
    };

}  // namespace grid
}  // namespace maps

#endif /* __MAPS_MLSMAPI_HPP__ */
