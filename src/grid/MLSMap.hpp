#ifndef __MAPS_MLS_GRID_HPP__
#define __MAPS_MLS_GRID_HPP__

#include <vector>
#include <set>
#include <exception>

#include <Eigen/Geometry>

#include <boost/scoped_ptr.hpp>

#include "MLGrid.hpp"
#include "MLSConfig.hpp"
#include "SurfacePatches.hpp"

#include <maps/tools/BresenhamLine.hpp>

namespace maps { namespace grid
{
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


    template<enum MLSConfig::update_model  SurfaceType>
    struct MLSMap : public MLGrid<SurfacePatch<SurfaceType> >
    {
    public:
        typedef SurfacePatch<SurfaceType> Patch;
        typedef LevelList<Patch>SPListST; // TODO rename
        typedef MLGrid<Patch> Base;
        
        MLSConfig config;

        MLSMap(
                const Vector2ui &num_cells,
                const Vector2d &resolution,
                const MLSConfig &config_)
        : Base(num_cells, resolution)
        , config(config_)
        {
            // TODO assert that config is compatible to SurfaceType ...
        }

        MLSMap()
        {
            // empty
        }

        void merge(const MLSMap& other)
        {
            // TODO
        }

        void mergePointCloud(const PointCloud& pc, const Eigen::Affine3d& pc2grid, bool withUncertainty = false)
        {
            // TODO change everything to float, if possible (requires refactoring Grid)

            const bool useNegative = false && config.useNegativeInformation; // TODO also check that grid is not empty?
            MLSMap tempGrid = useNegative
                            ? MLSMap(Base::getNumCells(), Base::getResolution(), config)
                            : MLSMap();
            MLSMap &workGrid = useNegative ? tempGrid : *this;

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
                Eigen::Vector3d pos_diff;
                Index idx;
                // TODO toGrid is expensive (involves applying another Transformation)
                if(Base::toGrid(pos, idx, pos_diff))
                {
                    if(useNegative && isCovered(Base::at(idx), pos_diff.z(), stdev))
                        continue;

                    workGrid.at(idx).update(Patch(pos_diff.cast<float>(), stdev), config);
                    if(useNegative)
                        coveredCells.insert(idx);
                }
            }
            if(useNegative)
            {
                Index origin; // Grid index of sensor
                Eigen::Vector3d orig_relative; // coordinates of sensor inside grid cell
                Base::toGrid(pc2grid.translation(), origin, orig_relative);

                maps::tools::Bresenham::Point orig = origin.cast<int>(); // Origin in Bresenham compatible format
                for(IndexSet::const_iterator it = coveredCells.begin(); it != coveredCells.end(); ++it)
                {
                    const SPListST &cell = workGrid.at(*it);
                    for(typename SPListST::const_iterator cit = cell.begin(); cit != cell.end(); ++cit)
                    {
                        // Merge cell into main grid
                        Base::at(*it).update(*cit, config);

                        // This block does actually the same for all cit:
                        maps::tools::Bresenham bresLine(orig, it->cast<int>());
                        maps::tools::Bresenham::Point diff_coord = it->cast<int>() - orig;
                        Eigen::DenseIndex maxDim;
                        diff_coord.cwiseAbs().maxCoeff(&maxDim);
                        double invDiff = 1.0/diff_coord[maxDim];

                        // get Range of individual patch:
                        float min_z, max_z;
                        cit->getRange(min_z, max_z);
                        float z_max = max_z - orig_relative.z();
                        float height = max_z - min_z;

                        maps::tools::Bresenham::Point next;
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


        void mergePoint(const Eigen::Vector3d& point)
        {
            Eigen::Vector3d pos;
            Index idx;
            if(Base::toGrid(point, idx, pos))
            {
                // TODO get stddev from config or by function parameter
                Base::at(idx).update(Patch(pos.cast<float>(), 0.1), config);
            }
        }
    };


    // TODO move this typedef somewhere more globally


    typedef MLSMap<MLSConfig::SLOPE> MLSMapSloped;
    typedef MLSMap<MLSConfig::KALMAN> MLSMapKalman;


} /* namespace grid */
} /* namespace maps */

#endif // __MAPS_MLS_GRID_HPP__
