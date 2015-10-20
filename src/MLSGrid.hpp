#ifndef __ENVIRE_MAPS_MLS_GRID_HPP__
#define __ENVIRE_MAPS_MLS_GRID_HPP__

#include "Grid.hpp"
#include "SPList.hpp"

namespace envire 
{
	namespace maps 
	{
		class MLSGrid : public Grid<SPList> 
		{
		public:
			typedef typename SPList::Cell::iterator CellItr;
			typedef typename SPList::Cell::const_iterator CellItrConst;

			MLSGrid(GridConfig grid_config, MLSConfiguration mls_config = MLSConfiguration()) 
				: Grid<SPList>(SPList(mls_config), grid_config),
				mls_config(mls_config)
				{}

			

		private:
			MLSConfiguration mls_config;


		};
	}
}

#endif // __ENVIRE_MAPS_MLS_GRID_HPP__