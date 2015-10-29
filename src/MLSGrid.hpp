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

            MLSGrid()
                : Grid<SPList>()
            {};

            MLSGrid(GridConfig grid_config, MLSConfig mls_config = MLSConfig()) 
                : Grid<SPList>(SPList(mls_config), grid_config),
                mls_config(mls_config)
            {}

            const MLSConfig& getConfig() const 
            { 
                return mls_config; 
            }

        private:
            MLSConfig mls_config;


        };
    }
}

#endif // __ENVIRE_MAPS_MLS_GRID_HPP__