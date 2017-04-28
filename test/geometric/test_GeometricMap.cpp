//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#define BOOST_TEST_MODULE GeometricTest
#include <boost/test/unit_test.hpp>

/** Element type **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/LineSegment.hpp>

/** Geometric Map **/
#include <maps/geometric/GeometricMap.hpp>

#include <boost/math/special_functions/fpclassify.hpp>

#include <chrono>

using namespace ::maps::geometric;

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
    BOOST_CHECK_EQUAL(geometric_copy->getEPSGCode(), geometric->getEPSGCode());
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
    BOOST_CHECK_EQUAL(geometric_copy->getEPSGCode(), geometric->getEPSGCode());
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

