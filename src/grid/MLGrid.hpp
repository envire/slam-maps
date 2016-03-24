#pragma once

#include "LevelList.hpp"
#include "GridMap.hpp"

namespace maps {
    

template <class P>
class MLGrid : public GridMap<LevelList<P> >
{
public:
    MLGrid(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const boost::shared_ptr<LocalMapData> &data) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>(), data)
    {
    };

    MLGrid(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution) : GridMap<LevelList<P> >(num_cells, resolution, LevelList<P>())
    {
    };
    
    
    class MLView : public GridMap<LevelList<const P *> >
    {
    public:
        MLView(const Vector2ui &num_cells,
               const Eigen::Vector2d &resolution) : GridMap<LevelList<const P *> >(num_cells, resolution, LevelList<const P *>())
        {
        };
        
        MLView() : GridMap<LevelList<const P *> >()
        {
        };
    };

    bool overlap( double a1, double a2, double b1, double b2 ) const
    {
        return 
            ((a1 <= b2) && (a2 >= b2)) ||
            ((a1 <= b1) && (a2 >= b1));
    }
    
    MLView intersectCuboid(const Eigen::AlignedBox3f& box) const
    {
        double minHeight = box.min().z();
        double maxHeight = box.max().z();

        
        Index minIdx;
        Index maxIdx;
        
        if(!this->toGrid(box.min().cast<double>(), minIdx))
        {
            return MLView();
        }
        if(!this->toGrid(box.max().cast<double>(), maxIdx))
        {
            return MLView();
        }
        
        MLView ret(maxIdx-minIdx, this->resolution);
        
        for(size_t x = minIdx.x();x < maxIdx.x(); x++)
        {
            for(size_t y = minIdx.y(); y < maxIdx.y(); y++)
            {
                Index curIdx(x,y);
                
                LevelList<const P *> &retList(ret.at(Index(curIdx - minIdx)));
                
                for(const P &p: this->at(curIdx))
                {
                    if(overlap(p.getMin(), p.getMax(), minHeight, maxHeight))
                        retList.insert(&p);
                }
            }
        }
        
        return ret;
    }

};

}
