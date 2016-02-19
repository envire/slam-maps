#ifndef __ENVIRE_MAPS_MLS_GRID_HPP__
#define __ENVIRE_MAPS_MLS_GRID_HPP__

#include <vector>

#include "GridMap.hpp"
#include "SPList.hpp"
#include "MLSConfig.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include "SurfacePatches.hpp"

#include <boost/scoped_ptr.hpp>
#include "LevelList.hpp"


namespace vizkit3d {
// Forward declaration for visualization:
class PatchesGeode;
} // namespace vizkit3d

namespace base
{
namespace samples
{
struct Pointcloud;
} /* namespace samples */
} /* namespace base */

namespace envire
{
namespace maps
{

// TODO move this typedef somewhere more globally
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class MLSMapExchange
{
public:
    typedef std::vector<SurfacePatchExchange> SPContainer;
    typedef SPContainer::iterator SPIterator;
    typedef SPContainer::const_iterator const_SPIterator;

    template<class It>
    struct CellView
    {
        It begin() const {return first;}
        It end() const {return last;}
        //! conversion constructor
        template<class It2>
        CellView(const CellView<It2>& cell) : first (cell.first), last (cell.last) { }
    private:
        friend class MLSMapExchange;
        CellView(const It & firs, const It & las) : first(firs),  last(las) { }
        It first, last;
    };


    CellView<const_SPIterator> at(Index idx) const
    {
        const size_t *begin = &startIdx.at(idx);
        const size_t *last =  &startIdx.at(Index(0,0)) + startIdx.numElements();
        const size_t end = (begin+1 == last) ? SPList.size() : begin[1]; // last element is stored in element after current

        return CellView<const_SPIterator>(SPList.begin()+ *begin, SPList.begin()+end);
    }

    CellView<SPIterator> at(Index idx)
    {
        const size_t *begin = &startIdx.at(idx);
        const size_t *last =  (&startIdx.at(Index(0,0))) + startIdx.numElements();
        const size_t end = (begin+1 == last) ? SPList.size() : begin[1]; // last element is stored in element after current

        return CellView<SPIterator>(SPList.begin()+ *begin, SPList.begin()+end);
    }


private:
    //! Storage vector
    SPContainer SPList;
    // TODO add optional vector for color, or other information?

    //! Map of start indices:
    GridMap<size_t> startIdx;
    // TODO usually uint32_t should be sufficient
    // TODO perhaps use std::pair<uint32_t, uint32_t> which would allow gaps in the storage container
};


class MLSGrid
{
    // Internal representation of map is hidden:
    class MLSBase;
    boost::scoped_ptr<MLSBase> map;

public:
    MLSGrid();
    MLSGrid(const Vector2ui &num_cells_,
            const Eigen::Vector2d &resolution_,
            const MLSConfig & config = MLSConfig());
    ~MLSGrid();
    MLSGrid(const MLSGrid& other);
    MLSGrid& operator=(const MLSGrid& other);

    /**
     * Joins two MLSMaps.
     * The maps must have compatible configurations.
     */
    void merge(const MLSGrid& other);

    /**
     * Merges a pointcloud with a given origin into the map
     */
    void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = true);

    void mergePoint(const Eigen::Vector3d& point);

    void visualize(vizkit3d::PatchesGeode& geode) const;

    void intersectCuboid(const Eigen::AlignedBox3f& box, MLSMapExchange& output) const;

    void getMap(MLSMapExchange & output) const;

    const Grid& getGrid() const;
    Grid& getGrid();
};


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
            ((a1 < b2) && (a2 > b2)) ||
            ((a1 < b1) && (a2 > b1));
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
                    
//                     std::cout << "Found Patch " << p.bar << std::endl;
//                     if(p > maxHeight)
//                     {
//                         std::cout << "is bigger than max height " << std::endl;
//                         break;
//                     }
//                     
//                     if(p > minHeight)
//                     {
//                         std::cout << "Insert ! " << std::endl;
//                         retList.insert(&p);
//                     }
                }
            }
        }
        
        return ret;
    }

};


} /* namespace maps */
} /* namespace envire */

#endif // __ENVIRE_MAPS_MLS_GRID_HPP__
