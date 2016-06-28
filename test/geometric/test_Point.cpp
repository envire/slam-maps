#define BOOST_TEST_MODULE GeometricTest
#include <boost/test/unit_test.hpp>

/** Element type **/
#include <maps/geometric/Point.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps::geometric;

BOOST_AUTO_TEST_CASE(test_point_constructor)
{
    Point<double, 2> p_a(-2, -3);
    Point<double, 2> p_b(4, 2);

    BOOST_CHECK_EQUAL(p_a.x(), -2);
    BOOST_CHECK_EQUAL(p_a.y(), -3);

    BOOST_CHECK_EQUAL(p_b.x(), 4);
    BOOST_CHECK_EQUAL(p_b.y(), 2);

    Point<double, 2> v((p_b - p_a).normalized());
    BOOST_CHECK_EQUAL(v.x(), 0.76822127959737585);
    BOOST_CHECK_EQUAL(v.y(), 0.64018439966447993);
    BOOST_CHECK_EQUAL(1.2, v[0]/v[1]);
    BOOST_CHECK_EQUAL(0.87605805059819342, atan(v[0]/v[1]));

}

BOOST_AUTO_TEST_CASE(test_point_typedefs)
{
    Point2d point_2d(0.00, 0.00);
    Point3d point_3d(0.00, 0.00, 0.00);

    Point2f point_2f(0.00, 0.00);
    Point3f point_3f(0.00, 0.00, 0.00);
}
