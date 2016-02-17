#ifndef SLAM_ENVIRE_MAPS_VIZ_STANDALONEVISUALIZER_HPP_
#define SLAM_ENVIRE_MAPS_VIZ_STANDALONEVISUALIZER_HPP_

#include <boost/scoped_ptr.hpp>


namespace envire
{
namespace maps
{

class MLSGrid;

class StandaloneVisualizer
{
    class Impl;
    boost::scoped_ptr<Impl> impl;
public:
    StandaloneVisualizer();
    ~StandaloneVisualizer();

    bool wait(int usecs = 1000);

    void updateData(const MLSGrid& mls);
};

} /* namespace maps */
} /* namespace envire */

#endif /* SLAM_ENVIRE_MAPS_VIZ_STANDALONEVISUALIZER_HPP_ */
