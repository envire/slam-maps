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
#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/OccupancyGridMap.hpp>

#include "../tools/GeneratePointclouds.hpp"
using namespace ::maps::grid;
#include <base/TimeMark.hpp>


BOOST_AUTO_TEST_CASE(occupancy_simulate_LIDAR)
{
    Eigen::Vector3d res(0.125, 0.125, 0.05);
    Vector2ui numCells(200, 200);

    OccupancyConfiguration config;
    OccupancyGridMap *grid = new OccupancyGridMap(numCells, res, config);

    /** Translate the local frame (offset) **/
    grid->getLocalFrame().translation() << 0.5*grid->getSize(), 0;

    typedef Eigen::Hyperplane<double, 3> Plane;
    maps::LIDARSimulator lidar(Eigen::VectorXd::LinSpaced(32, -16*M_PI/180, +16*M_PI/180), Eigen::VectorXd::LinSpaced(360, -M_PI, +M_PI));
    //    maps::LIDARSimulator lidar(Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
    std::vector<Plane> scene;
    {
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();

        for(int i=0; i<3; ++i)
        {
            scene.push_back(Plane(q * Eigen::Vector3d::Unit(i), -2.0));
            scene.push_back(Plane(q * Eigen::Vector3d::Unit(i), 2.0));
        }
    }

    StandaloneVisualizer app;

    Eigen::ArrayXXd ranges;
    PointCloud pointcloud;
    pointcloud.sensor_origin_ = Eigen::Vector4f::Zero();
    Eigen::Affine3d trafo;
    trafo.setIdentity();
    int loop = 0;
    while (app.wait(1000))
    {
        if(++loop & 1023) continue;
        trafo.translation().setRandom();
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();
        trafo.linear() = q.toRotationMatrix();
        //        std::cout << trafo.matrix() << std::endl;

        lidar.getRanges(ranges, scene, trafo, &pointcloud);

        base::TimeMark timer("grid->mergePointCloud");
        grid->mergePointCloud(pointcloud, trafo);
        std::cout << timer << std::endl;
        app.updateData(*grid);
    }

    delete grid;
}
