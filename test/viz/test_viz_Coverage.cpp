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
#include <boost/archive/polymorphic_binary_oarchive.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/MLSMap.hpp>

#include <maps/operations/CoverageMapGeneration.hpp>

using namespace ::maps::grid;

template<class MLSMap>
static void show_MLS(const MLSMap& mls)
{
    std::cout << "update finish" << std::endl;
    StandaloneVisualizer app;
    app.updateData(mls);

    while (app.wait(1000))
    {
    }
}



BOOST_AUTO_TEST_CASE(mls_loop)
{
    //    GridConfig conf(150, 150, 0.1, 0.1, -7.5, -7.5);
    Eigen::Vector2d res(0.1, 0.1);
    Vector2ui numCells(150, 150);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::KALMAN;
    mls_config.gapSize = 0.05f;
    mls_config.useNegativeInformation = false;
    float R = 5.0f, r=2.05f;
    MLSMapKalman mls(numCells, res, mls_config);

    maps::operations::CoverageTracker coverage;


    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;

    coverage.updateMLS(mls);

    StandaloneVisualizer app;

    Eigen::ArrayXXd ranges;
    PointCloud pointcloud;
    base::Pose trafo;
    int loop = 0;
    while (app.wait(1000))
    {
        if(++loop & 1023) continue;
        trafo.position +=Eigen::Vector3d::Random() * 0.5;
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();
        trafo.orientation = q;

        coverage.addCoverage(2.0, base::AngleSegment(), trafo);

        app.updateData(coverage.getCoverage());
    }
}

