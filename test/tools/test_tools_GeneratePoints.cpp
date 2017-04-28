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
#define BOOST_TEST_MODULE ToolsTest
#include <boost/test/unit_test.hpp>

#include "GeneratePointclouds.hpp"


using namespace ::maps;

BOOST_AUTO_TEST_CASE(test_generate_PC)
{
    typedef Eigen::Hyperplane<double, 3> Plane;
    LIDARSimulator lidar(Eigen::VectorXd::LinSpaced(20, -1.0, +1.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
//    LIDARSimulator lidar(Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
    std::vector<Plane> scene;
    for(int i=0; i<3; ++i)
    {
        scene.push_back(Plane(Eigen::Vector3d::Unit(i), 2.0));
        scene.push_back(Plane(Eigen::Vector3d::Unit(i), -2.0));
    }
    Eigen::ArrayXXd ranges;
    PointCloud pointcloud;
    Eigen::Affine3d trafo;
    trafo.setIdentity();
    for(int k=0; k<5; ++k)
    {
        trafo.translation().setRandom();
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();
        trafo.linear() = q.toRotationMatrix();
//        std::cout << trafo.matrix() << std::endl;

        lidar.getRanges(ranges, scene, trafo, &pointcloud);

        ranges.transposeInPlace();

        for(PointCloud::const_iterator it = pointcloud.begin(); it != pointcloud.end(); ++it)
        {
//            std::cout << ranges(it - pointcloud.begin()) << ",\t" << it->getVector3fMap().transpose() << std::endl;
            BOOST_CHECK_CLOSE((trafo * it->getVector3fMap().cast<double>()).lpNorm<Eigen::Infinity>(), 2.0, 0.01);
        }

    }
}


