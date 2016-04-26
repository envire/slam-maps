#pragma once

#include "LevelList.hpp"
#include "GridMap.hpp"

namespace maps { namespace grid
{

    template <class P>
    class MultiLevelGridMap : public GridMap<LevelList<P> >
    {
    public:
        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>(), data)
        {}

        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>())
        {}
        
        MultiLevelGridMap() {}
    };

}}
