#ifndef __MAPS_VIZ_STANDALONEVISUALIZER_HPP_
#define __MAPS_VIZ_STANDALONEVISUALIZER_HPP_

#include <boost/scoped_ptr.hpp>

#include <maps/grid/MLSMap.hpp>
#include <maps/grid/OccupancyGridMap.hpp>

namespace maps{ namespace grid
{

class StandaloneVisualizer
{
    class Impl;
    boost::scoped_ptr<Impl> impl;
public:
    StandaloneVisualizer();
    ~StandaloneVisualizer();

    bool wait(int usecs = 1000);

    void updateData(const ::maps::grid::MLSMapKalman& mls);
    void updateData(const ::maps::grid::MLSMapSloped& mls);
    void updateData(const ::maps::grid::OccupancyGridMap& grid);
};

} /* namespace grid */
} /* namespace maps */

#endif /* __MAPS_VIZ_STANDALONEVISUALIZER_HPP_ */
