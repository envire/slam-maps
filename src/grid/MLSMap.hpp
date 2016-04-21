#ifndef __MAPS_MLS_GRID_HPP__
#define __MAPS_MLS_GRID_HPP__

#include <vector>

#include "GridMap.hpp"
#include "MLSConfig.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include "SurfacePatches.hpp"

#include <boost/scoped_ptr.hpp>


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

namespace maps { namespace grid
{

// TODO move this typedef somewhere more globally
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#if 0

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

#else
// TODO implement
class MLSMapExchange;

#endif


template<enum MLSConfig::update_model SPType>
struct MLSMapI;

#if 0
class MLSMap
{
    // Internal representation of map is hidden:
    class MLSBase;
    boost::scoped_ptr<MLSBase> map;

public:
    MLSMap();
    MLSMap(const Vector2ui &num_cells_,
            const Eigen::Vector2d &resolution_,
            const MLSConfig & config = MLSConfig());
    ~MLSMap();
    MLSMap(const MLSMap& other);
    MLSMap& operator=(const MLSMap& other);

    /**
     * Joins two MLSMaps.
     * The maps must have compatible configurations.
     */
    void merge(const MLSMap& other);

    /**
     * Merges a pointcloud with a given origin into the map
     */
    void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = true);

    void mergePoint(const Eigen::Vector3d& point);

    void visualize(vizkit3d::PatchesGeode& geode) const;

    void intersectCuboid(const Eigen::AlignedBox3f& box, MLSMapExchange& output) const;

    void getMap(MLSMapExchange & output) const;

    Eigen::Vector2d getSize() const;
    base::Transform3d& getLocalFrame();
    Eigen::Vector2d getResolution() const;

    /** Low-level access to the actual MLSMap implementation  */
    template<class SPType>
    const MLSMapI<SPType>& getMLSMap() const;

    /** Low-level access to the actual MLSMap implementation  */
    template<class SPType>
    MLSMapI<SPType>& getMLSMap();
};

#endif


typedef MLSMapI<MLSConfig::SLOPE> MLSMapSloped;
typedef MLSMapI<MLSConfig::KALMAN> MLSMapKalman;


} /* namespace grid */
} /* namespace maps */
#include "MLSMapI.hpp"

#endif // __MAPS_MLS_GRID_HPP__
