#include <boost/test/unit_test.hpp>
#include <maps/geometric/GeometricMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps;

BOOST_AUTO_TEST_CASE(test_geometric_map_constructor)
{
    /** Check empty map **/
    GeometricMap *geometric = new GeometricMap();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    /** Check local map members **/
    BOOST_CHECK_EQUAL(geometric->getId(), ::maps::UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(geometric->getMapType(), ::maps::LocalMapType::GEOMETRIC_MAP);
    BOOST_CHECK_EQUAL(geometric->getEPSGCode(), ::maps::UNKNOWN_EPSG_CODE);
    BOOST_CHECK_EQUAL(geometric->getLocalFrame().translation(), base::Transform3d::Identity().translation());
    BOOST_CHECK_EQUAL(geometric->getLocalFrame().rotation(), base::Transform3d::Identity().rotation());

    delete geometric;
}
BOOST_AUTO_TEST_CASE(test_geometric_map_insert)
{

    /** Check empty map **/
    GeometricMap *geometric = new GeometricMap();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    Point<double> my_point(0.0, 0.0, 0.0);

    delete geometric;
}

BOOST_AUTO_TEST_CASE(test_geometric_map_copy)
{
    /** Check empty map **/
    GeometricMap *geometric = new GeometricMap();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    GeometricMap *geometric_copy = new GeometricMap(*geometric);

    // check configuration
    BOOST_CHECK_EQUAL(geometric_copy->getNumElements(), geometric->getNumElements());
    BOOST_CHECK_EQUAL(geometric_copy->getMapType(), geometric->getMapType());
    BOOST_CHECK_EQUAL(geometric->getEPSGCode(), geometric->getEPSGCode());
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().translation(), geometric->getLocalFrame().translation());    
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().rotation(), geometric->getLocalFrame().rotation());    

    delete geometric;
    delete geometric_copy;
}

