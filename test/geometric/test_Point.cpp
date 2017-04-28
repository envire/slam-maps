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
