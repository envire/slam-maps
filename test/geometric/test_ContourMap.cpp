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

