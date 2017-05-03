//
// Copyright (c) 2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2017, University of Bremen
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
#define BOOST_TEST_MODULE ToolsTest
#include <boost/test/unit_test.hpp>

#include <maps/tools/SurfaceIntersection.hpp>

#include <iostream>

#include <boost/test/results_collector.hpp>

inline bool current_test_passing()
{
  using namespace boost::unit_test;
  test_case::id_t id = framework::current_test_case().p_id;
  test_results rez = results_collector.results(id);
  return rez.passed();
}


BOOST_AUTO_TEST_CASE(test_surface_intersection)
{
    typedef Eigen::Vector3d Vec;
    typedef Eigen::AlignedBox3d Box;
    typedef Eigen::Hyperplane<double, 3> Plane;
    for(int i=0; i<100000; ++i)
    {
        // generate random box:
        Box box(Vec::Random());
        box.extend(Vec::Random());

        Vec inner = box.sample();
        BOOST_CHECK_SMALL(box.squaredExteriorDistance(inner), 1e-4);
        Vec normal = Vec::Random().normalized();
        BOOST_CHECK_CLOSE(normal.squaredNorm(), 1.0, 1e-6);
        Plane plane(normal, inner);

        std::vector<Vec> intersections;
        maps::tools::SurfaceIntersection::computeIntersections(plane, box, intersections);
        BOOST_CHECK_GE(intersections.size(), 3);
        BOOST_CHECK_LE(intersections.size(), 6);
        for(size_t j=0; j<intersections.size(); ++j)
        {
            const Vec &v = intersections[j];
            BOOST_CHECK_SMALL(plane.signedDistance(v), 1e-4); // v must be on the plane
            BOOST_CHECK_SMALL(box.squaredExteriorDistance(v), 1e-4); // v must not be outside the box

            // next point:
            const Vec &next = intersections[(j+1)%intersections.size()];
            BOOST_CHECK_SMALL((v-next).cwiseAbs().minCoeff(), 1e-6); // at least one coordinate must be equal to the next
        }

        // If anything failed, break and output input values:
        BOOST_REQUIRE_MESSAGE(current_test_passing(),
                "FAILED at i=" << i << " with box = [" << box.min().transpose() << " : " << box.max().transpose() << "], "
                        "plane = (" << normal.transpose() << "; " << inner.transpose() << ')');
    }
}
