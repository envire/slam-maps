#include <boost/test/unit_test.hpp>

/** Element type **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/LineSegment.hpp>

/** Geometric Map **/
#include <maps/geometric/GeometricMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps;

BOOST_AUTO_TEST_CASE(test_geometric_map_constructor)
{
    /** Check empty map **/
    GeometricMap< Point<double, 3> > *geometric = new GeometricMap< Point<double, 3> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    /** Check local map members **/
    BOOST_CHECK_EQUAL(geometric->getId(), ::maps::UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(geometric->getMapType(), ::maps::LocalMapType::GEOMETRIC_MAP);
    BOOST_CHECK_EQUAL(geometric->getEPSGCode(), ::maps::UNKNOWN_EPSG_CODE);
    BOOST_CHECK_EQUAL(geometric->getLocalFrame().translation(), base::Transform3d::Identity().translation());
    BOOST_CHECK_EQUAL(geometric->getLocalFrame().rotation(), base::Transform3d::Identity().rotation());

    delete geometric;
}
BOOST_AUTO_TEST_CASE(test_geometric_push_and_pop)
{

    /** Check empty map **/
    GeometricMap< Point<double, 3> > *geometric = new GeometricMap< Point<double, 3> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    Point<double, 3> my_point(0.0, 0.0, 0.0);

    geometric->push_back(my_point);

    BOOST_CHECK_EQUAL(geometric->getNumElements(), 1);

    geometric->pop_back();

    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    delete geometric;
}

BOOST_AUTO_TEST_CASE(test_geometric_insert)
{
    /** Check empty map **/
    GeometricMap< Point<double, 3> > *geometric = new GeometricMap< Point<double, 3> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);
    BOOST_CHECK_EQUAL(geometric->capacity(), 0);

    geometric->resize(100);
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 100);
    BOOST_CHECK_EQUAL(geometric->capacity(), 100);

    Point<double, 3> my_point(0.0, 0.0, 0.0);

    for (GeometricMap< Point<double, 3> >::iterator it = geometric->begin();
            it != geometric->end(); ++it)
    {
        (*it) << my_point;
    }

    my_point << 1.00, 1.00, 1.00;

    GeometricMap< Point<double, 3> >::iterator it = geometric->begin();
    geometric->insert(it, my_point);
    BOOST_CHECK_EQUAL(geometric->at(0), my_point);
    BOOST_CHECK_EQUAL((*geometric)[0], my_point);
}

BOOST_AUTO_TEST_CASE(test_geometric_erase)
{
    /** Check empty map **/
    GeometricMap< Point<double, 3> > *geometric = new GeometricMap< Point<double, 3> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    Point<double, 3> my_point(0.0, 0.0, 0.0);

    geometric->push_back(my_point);

    BOOST_CHECK_EQUAL(geometric->getNumElements(), 1);

    geometric->erase(geometric->begin());

    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    delete geometric;

}

BOOST_AUTO_TEST_CASE(test_geometric_3d_point_map_copy)
{
    /** Check empty map **/
    GeometricMap< Point<double, 3> > *geometric = new GeometricMap< Point<double, 3> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);
    Point<double, 3> my_point(0.0, 0.0, 0.0);

    for (register int i=0; i < 99 ; ++i)
    {
        geometric->push_back(my_point);
        //std::cout<<"my_point: "<<my_point[0]<<" "<<my_point[1]<<" "<<my_point[2]<<"\n";
        my_point.x()++;
    }


    GeometricMap< Point<double, 3> > *geometric_copy = new GeometricMap< Point<double, 3> >(*geometric);

    // check configuration
    BOOST_CHECK_EQUAL(geometric_copy->getNumElements(), geometric->getNumElements());
    BOOST_CHECK_EQUAL(geometric_copy->getMapType(), geometric->getMapType());
    BOOST_CHECK_EQUAL(geometric->getEPSGCode(), geometric->getEPSGCode());
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().translation(), geometric->getLocalFrame().translation());    
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().rotation(), geometric->getLocalFrame().rotation());    

    my_point << 0.00, 0.00, 0.00;
    for (register int i=0; i < 99 ; ++i)
    {
        BOOST_CHECK_EQUAL((*geometric_copy)[i], my_point);
        //std::cout<<"copy my_point: "<<(*geometric_copy)[i][0]<<" "<<(*geometric_copy)[i][1]<<" "<<(*geometric_copy)[i][2]<<"\n";
        my_point.x()++;
    }

    delete geometric;
    delete geometric_copy;
}

BOOST_AUTO_TEST_CASE(test_geometric_2d_line_map_copy)
{
    /** Check empty map **/
    GeometricMap< LineSegment<double, 2> > *geometric = new GeometricMap< LineSegment<double, 2> >();
    BOOST_CHECK_EQUAL(geometric->getNumElements(), 0);

    Point<double, 2> p_a(-2, -3);
    Point<double, 2> p_b(4, 2);
    LineSegment<double, 2> my_line(p_a, p_b);

    for (register int i=0; i < 99 ; ++i)
    {
        LineSegment<double, 2> my_line(p_a, p_b);
        geometric->push_back(my_line);
        p_a.x()++;
    }

    GeometricMap< LineSegment<double, 2> > *geometric_copy = new GeometricMap< LineSegment<double, 2> >(*geometric);

    // check configuration
    BOOST_CHECK_EQUAL(geometric_copy->getNumElements(), geometric->getNumElements());
    BOOST_CHECK_EQUAL(geometric_copy->getMapType(), geometric->getMapType());
    BOOST_CHECK_EQUAL(geometric->getEPSGCode(), geometric->getEPSGCode());
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().translation(), geometric->getLocalFrame().translation());
    BOOST_CHECK_EQUAL(geometric_copy->getLocalFrame().rotation(), geometric->getLocalFrame().rotation());

    p_a << -2.00, -3.00;
    for (register int i=0; i < 99 ; ++i)
    {
        LineSegment<double, 2> my_line(p_a, p_b);
        BOOST_CHECK_EQUAL((*geometric_copy)[i].psi_a(), my_line.psi_a());
        BOOST_CHECK_EQUAL((*geometric_copy)[i].psi_b(), my_line.psi_b());
        p_a.x()++;
    }

    delete geometric;
    delete geometric_copy;
}

