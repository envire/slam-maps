#define BOOST_TEST_MODULE GeometricTest
#include <boost/test/unit_test.hpp>

/** Geometric Contour Map **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/ContourMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps::geometric;

BOOST_AUTO_TEST_CASE(test_contour_map_constructor)
{
    /** Check empty map **/
    ContourMap *contour = new ContourMap();
    BOOST_CHECK_EQUAL(contour->getNumElements(), 0);

    /** Check local map members **/
    BOOST_CHECK_EQUAL(contour->getId(), ::maps::UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(contour->getMapType(), ::maps::LocalMapType::GEOMETRIC_MAP);
    BOOST_CHECK_EQUAL(contour->getEPSGCode(), ::maps::UNKNOWN_EPSG_CODE);
    BOOST_CHECK_EQUAL(contour->getLocalFrame().translation(), base::Transform3d::Identity().translation());
    BOOST_CHECK_EQUAL(contour->getLocalFrame().rotation(), base::Transform3d::Identity().rotation());

    delete contour;
}


BOOST_AUTO_TEST_CASE(test_contour_map_copy)
{
    /** Check empty map **/
    ContourMap *contour = new ContourMap();
    BOOST_CHECK_EQUAL(contour->getNumElements(), 0);

    Point3d p_a, p_b;
    p_a << 0.00, 0.00, 0.00; p_b << 1.00, 1.00, 1.00;
    LineSegment3d my_line(p_a, p_b);

    for (register unsigned int i=0; i < 99 ; ++i)
    {
        contour->push_back(my_line);
        my_line.psi_a() << p_a.x()++, p_a.y(), p_a.z();
    }

    ContourMap *contour_copy = new ContourMap(*contour);

    // check configuration
    BOOST_CHECK_EQUAL(contour_copy->getNumElements(), contour->getNumElements());
    BOOST_CHECK_EQUAL(contour_copy->getMapType(), contour->getMapType());
    BOOST_CHECK_EQUAL(contour->getEPSGCode(), contour->getEPSGCode());
    BOOST_CHECK_EQUAL(contour_copy->getLocalFrame().translation(), contour->getLocalFrame().translation());
    BOOST_CHECK_EQUAL(contour_copy->getLocalFrame().rotation(), contour->getLocalFrame().rotation());

    for (register unsigned int i=0; i < contour->getNumElements() ; ++i)
    {
        BOOST_CHECK_EQUAL((*contour_copy)[i].psi_a(), (*contour)[i].psi_a());
        BOOST_CHECK_EQUAL((*contour_copy)[i].psi_b(), (*contour)[i].psi_b());
    }

    delete contour;
    delete contour_copy;
}

