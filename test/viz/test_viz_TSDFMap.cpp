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
#include <maps/grid/TSDFVolumetricMap.hpp>
#include <maps/tools/TSDF_MLSMapReconstruction.hpp>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(tsdf_moving_waves)
{
    Vector3d res(0.2, 0.2, 0.2);
    Vector2ui numCells(40, 40);

    StandaloneVisualizer app;

    TSDFVolumetricMap::Ptr grid(new TSDFVolumetricMap(numCells, res));

    /** Translate the local frame (offset) **/
    grid->getLocalFrame().translation() << 0.5*grid->getSize(), 0;

    maps::tools::TSDF_MLSMapReconstruction mls_reconstruction;
    mls_reconstruction.setTSDFMap(grid);

    Eigen::Vector2d max = 0.375 * grid->getSize();
    Eigen::Vector2d min = -0.375 * grid->getSize();

    double x_shift = 0.;
    while (app.wait(200))
    {
        x_shift += 0.01;
        for (double x = min.x(); x < max.x(); x += 0.05)
        {
            double cs = std::cos(x_shift + x * M_PI/2.5);
            for (double y = min.y(); y < max.y(); y += 0.05)
            {
                double sn = std::sin(y * M_PI/2.5);
                try
                {
                    grid->mergePoint(Eigen::Vector3d(x, y, 4.) + Eigen::Vector3d::Random(), Eigen::Vector3d(x, y, cs*sn));
                }
                catch(std::runtime_error &e)
                {
                    std::cerr << "Caught exception: " << e.what() << std::endl;
                }
            }
        }

        maps::grid::MLSMapPrecalculated mls;
        mls_reconstruction.reconstruct(mls);
        app.updateData(mls);
    }
}
