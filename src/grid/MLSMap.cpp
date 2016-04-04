#include "MLSMap.hpp"

#include <maps/tools/BresenhamLine.hpp>

#include "GridMap.hpp"

#include <exception>

#include "MLSMapI.hpp"


namespace maps {

template<class Patch>
bool isCovered(const LevelList<Patch> &list, float zPos, const float gapSize = 0.0)
{
    for(typename LevelList<Patch>::const_iterator it = list.begin(); it!= list.end(); ++it)
    {
        if(it->isCovered(zPos, gapSize)) return true;
    }
    return false;
}

template<class Patch>
bool merge(Patch& a, const Patch& b, const MLSConfig& config)
{
    return a.merge(b, config);
}

template<class Patch>
void LevelList<Patch>::update(const Patch& o, const MLSConfig& conf)
{
    typedef typename LevelList<Patch>::iterator iterator;
    iterator it = this->begin(), end = this->end(), it_prev = end;
    if(it==end)
    {
        // insert into empty list
        this->insert(o);
        return;
    }
    // Find iterators so that *it_prev < o < *it
    while(it != end && *it < o)
    {
        it_prev=it;
        ++it;
    }
    if(it_prev == end)
    {
        // new patch is smaller than first patch
        if(!merge(*it, o, conf))
            this->insert(o);
    }
    else if(it==end)
    {
        // new patch is larger than last patch
        if(!merge(*it_prev, o, conf))
            this->insert(o);
    }
    else
    {
        // new patch lies between it_prev and it
        // try to merge with previous patch:
        if(merge(*it_prev, o, conf))
        {
            // this might make this patch merge-able with the next patch
            if(merge(*it_prev, *it, conf))
            {
                // erase the second patch, since it was merged with the first
                this->erase(it);
            }
        }
        // otherwise, try to merge with the next patch
        else if(!merge(*it, o, conf))
        {
            // if it is not merge-able, insert as a new patch between existing patches
            this->insert(it, o);
        }
    }

}


#define MAPS_MLSMAP(ret_type__) template<enum MLSConfig::update_model SurfaceType> \
    ret_type__ MLSMapI<SurfaceType>

MAPS_MLSMAP(void)::mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty)
    {
        // TODO change everything to float, if possible (requires refactoring Grid)

        const bool useNegative = false && config.useNegativeInformation; // TODO also check that grid is not empty?
        MLSMapI tempGrid = useNegative
                        ? MLSMapI(Base::getNumCells(), Base::getResolution(), config)
                        : MLSMapI();
        MLSMapI &workGrid = useNegative ? tempGrid : *this;

    //    const std::vector<Eigen::Vector3d> & points = pc.vertices;
        // TODO uncertainty and color are ignored for now
        const double p_var = 0.01; // default uncertainty
        const double stdev = std::sqrt(p_var);
        typedef std::set<Index> IndexSet;
        IndexSet coveredCells;

        for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
        {
            Eigen::Vector3d point = it->getArray3fMap().cast<double>();
            Eigen::Vector3d pos = pc2grid * point;
            Index idx;
            // TODO toGrid is expensive (involves applying another Transformation)
            if(Base::toGrid(pos, idx, pos))
            {
                if(useNegative && isCovered(Base::at(idx), pos.z(), stdev))
                    continue;

                workGrid.at(idx).update(Patch(pos.cast<float>(), stdev), config);
                if(useNegative)
                    coveredCells.insert(idx);
            }
        }
        if(useNegative)
        {
            Index origin; // Grid index of sensor
            Eigen::Vector3d orig_relative; // coordinates of sensor inside grid cell
            Base::toGrid(pc2grid.translation(), origin, orig_relative);

            Bresenham::Point orig = origin.cast<int>(); // Origin in Bresenham compatible format
            for(IndexSet::const_iterator it = coveredCells.begin(); it != coveredCells.end(); ++it)
            {
                const SPListST &cell = workGrid.at(*it);
                for(typename SPListST::const_iterator cit = cell.begin(); cit != cell.end(); ++cit)
                {
                    // Merge cell into main grid
                    Base::at(*it).update(*cit, config);

                    // This block does actually the same for all cit:
                    Bresenham bresLine(orig, it->cast<int>());
                    Bresenham::Point diff_coord = it->cast<int>() - orig;
                    Eigen::DenseIndex maxDim;
                    diff_coord.cwiseAbs().maxCoeff(&maxDim);
                    double invDiff = 1.0/diff_coord[maxDim];

                    // get Range of individual patch:
                    float min_z, max_z;
                    cit->getRange(min_z, max_z);
                    float z_max = max_z - orig_relative.z();
                    float height = max_z - min_z;

                    Bresenham::Point next;
                    while(bresLine.getNextPoint(next))
                    {
                        double factor = (next-orig)[maxDim] * invDiff;
                        if(factor <= 0 || factor >= 1.0) continue;
                        const double p_var = 0.05;
                        double z_mean = z_max * (factor) + orig_relative.z() - p_var;
                        double p_height = height * (factor) - 2*p_var;
                        //double z_stdev = cit->getStdev() * factor;
                        double z_stdev = 0.05 * factor; // TODO get stdev from patch
                        Patch np(z_mean, z_stdev, p_height, Patch::NEGATIVE);
                        // TODO set update_idx of np?

                        // merge negative patch:
                        Base::at(Index(next.cast<Index::Scalar>())).update(np, config);
                    }
                }
            }
        }
    }


MAPS_MLSMAP(void)::mergePoint(const Vector3d & point)
{
    Eigen::Vector3d pos;
    Index idx;
    if(Base::toGrid(point, idx, pos))
    {
        // TODO get stddev from config or by function parameter
        Base::at(idx).update(Patch(pos.cast<float>(), 0.1), config);
    }
}

#if 0
MLSMap::MLSMap(
        const Vector2ui &num_cells_,
        const Eigen::Vector2d &resolution_,
        const MLSConfig & config)
{
    switch(config.updateModel)
    {
    case MLSConfig::SLOPE:
        map.reset(new MLSMapI<SurfacePatchT<MLSConfig::SLOPE> >(num_cells_, resolution_, config));
        break;
    case MLSConfig::KALMAN:
        map.reset(new MLSMapI<SurfacePatchT<MLSConfig::KALMAN> >(num_cells_, resolution_, config));
        break;
    default:
        throw std::runtime_error("Not implemented!");
        break;
    }
}
MLSMap::MLSMap()
{
    // empty
}

MLSMap::~MLSMap()
{
    // empty
}
MLSMap::MLSMap(const MLSMap& other) : map(other.map->clone())
{ }

void MLSMap::mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty)
{
    map->mergePointCloud(pc, pc2grid, withUncertainty);
}

void MLSMap::mergePoint(const Eigen::Vector3d& point)
{
    map->mergePoint(point);
}


MLSMap& MLSMap::operator =(const MLSMap& other)
{
    if(this != &other)
        map.reset(other.map->clone());
    return *this;
}

Eigen::Vector2d MLSMap::getResolution() const
{
    return map->getResolution();
}
Eigen::Vector2d MLSMap::getSize() const
{
    return map->getSize();
}
base::Transform3d& MLSMap::getLocalFrame()
{
    return map->getLocalFrame();
}

template<class SPType>
const MLSMapI<SPType>& MLSMap::getMLSMap() const
{
    return dynamic_cast<const MLSMapI<SPType>&>(*map);
}
template<class SPType>
MLSMapI<SPType>& MLSMap::getMLSMap()
{
        return dynamic_cast<MLSMapI<SPType>&>(*map);
}
#endif

template struct MLSMapI<MLSConfig::KALMAN>;
template struct MLSMapI<MLSConfig::SLOPE>;


}  // namespace maps
