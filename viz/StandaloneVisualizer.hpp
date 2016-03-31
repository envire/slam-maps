#ifndef __MAPS_VIZ_STANDALONEVISUALIZER_HPP_
#define __MAPS_VIZ_STANDALONEVISUALIZER_HPP_

#include <boost/scoped_ptr.hpp>

#include <maps/grid/MLSMap.hpp>

namespace maps
{

class MLSMap;

class StandaloneVisualizer
{
    class Impl;
    boost::scoped_ptr<Impl> impl;
public:
    StandaloneVisualizer();
    ~StandaloneVisualizer();

    bool wait(int usecs = 1000);

    void updateData(const MLSMapKalman& mls);
    void updateData(const MLSMapSloped& mls);
};

} /* namespace maps */

#endif /* __MAPS_VIZ_STANDALONEVISUALIZER_HPP_ */
