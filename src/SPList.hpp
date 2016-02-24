#ifndef __ENVIRE_MAPS_SPLIST_HPP__
#define __ENVIRE_MAPS_SPLIST_HPP__

#include <boost/multi_array.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/pool/object_pool.hpp>

#include "List.hpp"
#include "SurfacePatch.hpp"

namespace envire 
{
    namespace maps 
    {
        template<class SurfacePatch>
        class SPList : public List<SurfacePatch> 
        {
        public:
            typedef List<SurfacePatch> Base;
            using Base::begin;
            using Base::end;
            using Base::insertTail;
            using Base::insertHead;
            using Base::erase;
            using Base::insert;

            typedef typename Base::Holder::iterator iterator;
            typedef typename Base::Holder::const_iterator const_iterator;

            SPList(MLSConfig config = MLSConfig()) 
                : List<SurfacePatch>(),
                config(config)
            {}

            void update(const SurfacePatch& co)
            {
                // make a copy of the surfacepatch as it may get updated in the merge
                // TODO: This can only happen if negative patches are involved, otherwise this copy could be avoided
                SurfacePatch o( co );

                iterator it = begin(), it_prev=end();
                if(it==end())
                {
                    // insert into empty list
                    insertTail(o);
                    return;
                }
                // Find iterators so that *it_prev < o < *it
                while(it != end() && *it < o)
                {
                    it_prev=it;
                    ++it;
                }
                if(it_prev == end())
                {
                    // new patch is smaller than first patch
                    if(!merge(*it, o))
                        insertHead(o);
                }
                else if(it==end())
                {
                    // new patch is larger than last patch
                    if(!merge(*it_prev, o))
                        insertTail(o);
                }
                else
                {
                    // new patch lies between it_prev and it
                    // try to merge with previous patch:
                    if(merge(*it_prev, o))
                    {
                        // this might make this patch merge-able with the next patch
                        if(merge(*it_prev, *it))
                        {
                            // erase the second patch, since it was merged with the first
                            erase(it);
                        }
                    }
                    // otherwise, try to merge with the next patch
                    else if(!merge(*it, o))
                    {
                        // if it is not merge-able, insert as a new patch between existing patches
                        insert(it, o);
                    }
                }

            }   
#if 0

            /** Finds a surface patch at \c (position.x, position.y) that matches
             * the Z information contained in \c patch (patch is used to get mean
             * and sigma Z).
             *
             * The mean Z of the returned patch has to be within \c sigma_threshold
             * patch.sigma of sigma.mean
             */
            iterator getPatchByZ(double zpos, double zstdev, double sigma_threshold = 3.0, bool ignore_negative = true)
            {
                SurfacePatch tmp(zpos, zstdev);
                return getPatchByZ(tmp, sigma_threshold, ignore_negative);
            }

            const_iterator getPatchByZ(double zpos, double zstdev, double sigma_threshold = 3.0, bool ignore_negative = true) const
            {
                SurfacePatch tmp(zpos, zstdev);
                return getPatchByZ(tmp, sigma_threshold, ignore_negative);
            }            

            bool isCovered(double zPos, double zStdev, double gapSize = 0.0) const
            {
                const SurfacePatch tmp(zPos, zStdev);
                return getPatchByZ(tmp)->isCovered(tmp, gapSize);
            }
        private:
            template<class It>
            It getPatchByZ(It it, It end, const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true) const
            {
                It found_patch = end;
                float nearest_distance = std::numeric_limits<float>::max();
                // TODO take advantage of the fact that patches are sorted
                for (;it != end; ++it)
                {                   
                    const SurfacePatch &p(*it);
                    const double interval = sqrt(sq(patch.getStdev()) + sq(p.getStdev())) * sigma_threshold;
                    const float distance = p.distance( patch );
                    if( distance < interval && distance < nearest_distance && (!ignore_negative || !p.isNegative()) )
                    {
                        nearest_distance = distance;
                        found_patch = it;
                    }
                }
                return found_patch;
            }
        public:
            iterator getPatchByZ(const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true)
            {
                return getPatchByZ(begin(), end(), patch, sigma_threshold, ignore_negative);
            }

            const_iterator getPatchByZ(const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true) const
            {
                return getPatchByZ(begin(), end(), patch, sigma_threshold, ignore_negative);
            }
#else
            bool isCovered(const float zPos, const float gapSize = 0.0) const
            {
                for(const_iterator it = begin(); it!= end(); ++it)
                {
                    if(it->isCovered(zPos, gapSize)) return true;
                }
                return false;
            }
#endif

            std::pair<iterator, double> getNearestPatch(const SurfacePatch& p)
            {
                iterator it_min = end();
                double dist = std::numeric_limits<double>::infinity();

                // find the cell with the smallest z-diff
                for (iterator it = begin(); it != end(); ++it)
                {                   
                    double d;
                    if( (d = p.distance(*it)) < dist )
                    {
                        it_min = it;
                        dist = d;
                    }
                }

                return std::make_pair(it_min, dist);                
            }

        protected:

            MLSConfig config;

            bool merge(SurfacePatch& p, SurfacePatch& o)
            {
                return p.merge(o, config.gapSize);
            }                   
        };

    }
}

#endif // __ENVIRE_MAPS_SPLIST_HPP__
