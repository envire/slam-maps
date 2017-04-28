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

template<MLSConfig::update_model mlsType>
void mls_waves(const std::string& filename)
{
    Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    typedef MLSMap<mlsType> MLSMap;
    mls_config.updateModel = mlsType;
    MLSMap *mls = new MLSMap(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls->getLocalFrame().translation() << 0.5*mls->getSize(), 0;

    /** Equivalent to translate the grid in opposite direction **/
    //Eigen::Vector3d offset_grid;
    //offset_grid << -0.5*mls->getSize(), 0.00;
    //mls->translate(offset_grid);

    Eigen::Vector2d max = 0.5 * mls->getSize();
    Eigen::Vector2d min = -0.5 * mls->getSize();
    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double sn = std::sin(y * M_PI/2.5);
            mls->mergePoint(Eigen::Vector3d(x, y, cs*sn));
        }
    }

    if(!filename.empty())
    {
        std::ofstream of(filename.c_str(), std::ios::binary);
        boost::archive::binary_oarchive oa(of);
        oa << *mls;
    }

    show_MLS(*mls);

}

BOOST_AUTO_TEST_CASE( mlsviz_test )
{
    mls_waves<MLSConfig::SLOPE>("MLSMapSloped_waves.bin");
    mls_waves<MLSConfig::KALMAN>("MLSMapKalman_waves.bin");
}

BOOST_AUTO_TEST_CASE(mls_loop)
{
    //    GridConfig conf(150, 150, 0.1, 0.1, -7.5, -7.5);
    Eigen::Vector2d res(0.1, 0.1);
    Vector2ui numCells(150, 150);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //    mls_config.updateModel = MLSConfig::KALMAN;
    mls_config.gapSize = 0.05f;
    mls_config.useNegativeInformation = false;
    float R = 5.0f, r=2.05f;
    MLSMapSloped *mls = new MLSMapSloped(numCells, res, mls_config);
    mls->getLocalFrame().translation() << 0.5*mls->getSize(), 0;

    for (float alpha = 0; alpha < M_PI; alpha += M_PI/1024/4)
    {
        float cs = std::cos(alpha);
        float sn = std::sin(alpha);
        for (float beta = 0; beta <= 2*M_PI; beta+=M_PI/256/4)
        {
            // Points of a torus:
            float x = (R+r*std::cos(beta)) * cs;
            float y = (R+r*std::cos(beta)) * sn;
            float z = r*std::sin(beta);

            mls->mergePoint(Vector3d(x,y,z));

        }
    }
    std::ofstream of("MLSMapSloped_loop.bin", std::ios::binary);
    boost::archive::binary_oarchive oa(of);
    oa << *mls;

    show_MLS(*mls);
}

