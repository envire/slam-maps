#pragma once

#include "LevelList.hpp"
#include "GridMap.hpp"

namespace maps { namespace grid
{

    template <class P>
    class MLGrid : public GridMap<LevelList<P> >
    {
    public:
        MLGrid(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>(), data)
        {}

        MLGrid(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>())
        {}
        
        MLGrid() {}
    };

}}
