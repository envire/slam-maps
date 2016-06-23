#pragma once

#include "LevelList.hpp"
#include "GridMap.hpp"
#include "../tools/Overlap.hpp"

namespace maps { namespace grid
{

    template <class P>
    class MultiLevelGridMap : public GridMap<LevelList<P> >
    {
    public:
        typedef LevelList<P> CellType; 
        
        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution,
                    const boost::shared_ptr<LocalMapData> &data) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>(), data)
        {}

        MultiLevelGridMap(const Vector2ui &num_cells,
                    const Eigen::Vector2d &resolution) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>())
        {}
        
        MultiLevelGridMap() {}
        
        class View : public GridMap<LevelList<const P *> >
        {
        public:
            View(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution) : GridMap<LevelList<const P *> >(num_cells, resolution, LevelList<const P *>())
            {
            };
            
            View() : GridMap<LevelList<const P *> >()
            {
            };
        };
    
        View intersectCuboid(const Eigen::AlignedBox3d& box) const
        {
            double minHeight = box.min().z();
            double maxHeight = box.max().z();

            
            Index minIdx;
            Index maxIdx;
            
            if(!this->toGrid(box.min(), minIdx))
            {
                return View();
            }

            if(!this->toGrid(box.max(), maxIdx))
            {
                return View();
            }
            
            View ret(maxIdx-minIdx, this->getResolution());
            
            for(size_t x = minIdx.x();x < maxIdx.x(); x++)
            {
                for(size_t y = minIdx.y(); y < maxIdx.y(); y++)
                {
                    Index curIdx(x,y);
                    
                    LevelList<const P *> &retList(ret.at(Index(curIdx - minIdx)));
                    
                    for(const P &p: this->at(curIdx))
                    {
                        if(::maps::tools::overlap(p.getMin(), p.getMax(), minHeight, maxHeight))
                        {
                            retList.insert(&p);
                        }
                    }
                }
            }
            
            return ret;
        }
        
    };

}}
