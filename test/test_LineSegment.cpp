#include <boost/test/unit_test.hpp>

/** Element type **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/LineSegment.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps::geometric;

BOOST_AUTO_TEST_CASE(test_linesegment_constructor)
{
    Point<double, 2> p_a(-2, -3);
    Point<double, 2> p_b(4, 2);
    LineSegment<double, 2> line(p_a, p_b);

    BOOST_CHECK_EQUAL(line.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line.psi_b(), p_b);

}

BOOST_AUTO_TEST_CASE(test_linesegment_2d)
{
    Point<double, 2> p_a(0, 6);
    Point<double, 2> p_b(6, 6);
    LineSegment<double, 2> line_0(p_a, p_b);

    BOOST_CHECK_EQUAL(line_0.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line_0.psi_b(), p_b);
    BOOST_CHECK_EQUAL(line_0.rho(), 6);
    BOOST_CHECK_EQUAL(line_0.alpha(), M_PI/2.0);

    p_a << 0.00, 0.00; p_b << 6.00, 6.00;
    LineSegment<double, 2> line_1(p_a, p_b);

    BOOST_CHECK_EQUAL(line_1.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line_1.psi_b(), p_b);
    BOOST_CHECK_EQUAL(line_1.rho(), 0);
    BOOST_CHECK_EQUAL(line_1.alpha(), 3.0*(M_PI/4.0));

    p_a << 0.00, 6.00; p_b << 6.00, 0.00;
    LineSegment<double, 2> line_2(p_a, p_b);

    BOOST_CHECK_EQUAL(line_2.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line_2.psi_b(), p_b);
    BOOST_CHECK_EQUAL(line_2.rho(), 4.2426406871192848);
    BOOST_CHECK_EQUAL(line_2.alpha(), M_PI/4.0);

    p_a << -6.00, 0.00; p_b << 0.00, 6.00;
    LineSegment<double, 2> line_3(p_a, p_b);

    BOOST_CHECK_EQUAL(line_3.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line_3.psi_b(), p_b);
    BOOST_CHECK_EQUAL(line_3.rho(), 4.2426406871192848);
    BOOST_CHECK_EQUAL(line_3.alpha(), 3.0*(M_PI/4.0));

    //std::cout<<"rho = "<<line_3.rho()<<"\n";
    //std::cout<<"alpha = "<<line_3.alpha()<<"\n";

}


BOOST_AUTO_TEST_CASE(test_linesegment_3d)
{
    Point<double, 3> p_a(0, 6, 0);
    Point<double, 3> p_b(6, 6, 0);
    LineSegment<double, 3> line_0(p_a, p_b);

    BOOST_CHECK_EQUAL(line_0.psi_a(), p_a);
    BOOST_CHECK_EQUAL(line_0.psi_b(), p_b);
    BOOST_CHECK_EQUAL(line_0.rho(), 6);
    BOOST_CHECK_EQUAL(boost::math::isnan(line_0.alpha()), true);
}

BOOST_AUTO_TEST_CASE(test_linesegment_typedefs)
{
    Point<double, 2> p_a_2d(0, 6);
    Point<double, 2> p_b_2d(6, 6);

    Point<double, 3> p_a_3d(0, 6, 0);
    Point<double, 3> p_b_3d(6, 6, 0);

    LineSegment2d line_2d(p_a_2d, p_b_2d);
    LineSegment3d line_3d(p_a_3d, p_b_3d);

    Point<float, 2> p_a_2f(0, 6);
    Point<float, 2> p_b_2f(6, 6);

    Point<float, 3> p_a_3f(0, 6, 0);
    Point<float, 3> p_b_3f(6, 6, 0);

    LineSegment2f line_2f(p_a_2f, p_b_2f);
    LineSegment3f line_3f(p_a_3f, p_b_3f);
}


