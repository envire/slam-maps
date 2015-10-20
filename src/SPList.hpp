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
		class SPList : public List<SurfacePatch> 
		{
		public:
			typedef typename List<SurfacePatch>::Cell::iterator CellItr;
			typedef typename List<SurfacePatch>::Cell::const_iterator CellItrConst;

			SPList(MLSConfiguration config = MLSConfiguration()) 
				: List<SurfacePatch>(),
				config(config)
			{}

			void update(const SurfacePatch& co)
			{
				typedef std::list<CellItr> iterator_list;
			    iterator_list merged;

			    // make a copy of the surfacepatch as it may get updated in the merge
			    SurfacePatch o( co );

			    for (CellItr it = begin(); it != end(); it++)
			    {
					// merge the patches and remember the ones which where merged 
					if (merge(*it, o))
				    	merged.push_back(it);
			    }

			    if (merged.empty())
			    {
					// insert the patch since we didn't merge it with any other
					insertHead(o);
			    } else {
					// if there is more than one affected patch, merge them until 
					// there is only one left
					while (!merged.empty())
					{
					    iterator_list::iterator it = ++merged.begin();
					    while( it != merged.end() ) 
					    {
							if(merge(**merged.begin(), **it))
							{
							    erase(*it);
							    it = merged.erase(it);
							}
							else
							    it++;
					    }
					    merged.pop_front();
					}
			    }				
			}	

	        /** Finds a surface patch at \c (position.x, position.y) that matches
	         * the Z information contained in \c patch (patch is used to get mean
	         * and sigma Z).
	         *
	         * The mean Z of the returned patch has to be within \c sigma_threshold
	         * patch.sigma of sigma.mean
	         */
			CellItr getPatchByZ(double zpos, double zstdev, double sigma_threshold = 3.0, bool ignore_negative = true)
			{
			    SurfacePatch tmp(zpos, zstdev);
			    return getPatchByZ(tmp, sigma_threshold, ignore_negative);
			}

			CellItr getPatchByZ(const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true)
			{
				CellItr it = begin();
				for (;it != end(); ++it)
				{					
					SurfacePatch &p(*it);
					const double interval = sqrt(sq(patch.stdev) + sq(p.stdev)) * sigma_threshold;
					if( p.distance( patch ) < interval && (!ignore_negative || !p.isNegative()) )
					{
					    return it;
					}
			    }
			    return it;
			}

			std::pair<CellItr, double> getNearestPatch(const SurfacePatch& p)
			{
				CellItr it_min = end();
				double dist = std::numeric_limits<double>::infinity();

				// find the cell with the smallest z-diff
				for (CellItr it = begin(); it != end(); ++it)
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

			MLSConfiguration config;

			bool merge(SurfacePatch& p, SurfacePatch& o)
			{
			    return p.merge(o, config);
			}					
		};

	}
}

#endif // __ENVIRE_MAPS_SPLIST_HPP__