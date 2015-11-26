#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_empty)
{
    Grid grid(Eigen::Vector2d(0.1, 0.1), Vector2ui(100, 100));
}