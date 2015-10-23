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
			typedef typename List<SurfacePatch>::Holder::iterator iterator;
			typedef typename List<SurfacePatch>::Holder::const_iterator const_iterator;

			SPList(MLSConfig config = MLSConfig()) 
				: List<SurfacePatch>(),
				config(config)
			{}

			void update(const SurfacePatch& co)
			{
				typedef std::list<iterator> iterator_list;
			    iterator_list merged;

			    // make a copy of the surfacepatch as it may get updated in the merge
			    SurfacePatch o( co );

			    for (iterator it = begin(); it != end(); it++)
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
			iterator getPatchByZ(double zpos, double zstdev, double sigma_threshold = 3.0, bool ignore_negative = true)
			{
			    SurfacePatch tmp(zpos, zstdev);
			    return getPatchByZ(tmp, sigma_threshold, ignore_negative);
			}

			iterator getPatchByZ(const SurfacePatch& patch, double sigma_threshold = 3.0, bool ignore_negative = true)
			{
				iterator it = begin();
				for (;it != end(); ++it)
				{					
					SurfacePatch &p(*it);
					const double interval = sqrt(sq(patch.getStdev()) + sq(p.getStdev())) * sigma_threshold;
					if( p.distance( patch ) < interval && (!ignore_negative || !p.isNegative()) )
					{
					    return it;
					}
			    }
			    return it;
			}

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
			    return p.merge(o, config);
			}					
		};

	}
}

#endif // __ENVIRE_MAPS_SPLIST_HPP__