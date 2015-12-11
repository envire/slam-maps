#ifndef __ENVIRE_MAPS_MLS_GRID_HPP__
#define __ENVIRE_MAPS_MLS_GRID_HPP__

#include "GridMap.hpp"
#include "SPList.hpp"

namespace envire 
{
    namespace maps 
    {
        class MLSGrid : public GridMap<SPList>
        {
        public:

            MLSGrid()
                : GridMap<SPList>()
            {};

            MLSGrid( const Eigen::Vector2d &resolution_,
                    const Vector2ui &num_cells_ , MLSConfig mls_config = MLSConfig())
                : GridMap<SPList>(resolution_, num_cells_, SPList(mls_config)),
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
