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

#include <vizkit3d/StandaloneVisualizer.hpp>

#include <maps/grid/TraversabilityGrid.hpp>
#include <maps/grid/MLSMap.hpp>

#include <maps/tools/MLSToSlopes.hpp>
#include <maps/tools/SimpleTraversability.hpp>

using namespace ::maps;
using namespace ::grid;


TraversabilityGrid sinesMlsToTravGrid(const Vector2ui& numCells,const Vector2d& resolution)
{
    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::KALMAN;
    MLSMapKalman mls(numCells, resolution, mls_config);

    Eigen::Vector2d size = mls.getSize();
    float stepSizeX = resolution.x() / 10;
    float stepSizeY = resolution.y() / 10;
    for (double y = 0; y < size.y(); y += stepSizeY)
    {
        double cs = std::cos(y * M_PI/2.5);
        for (double x = 0; x < size.x(); x += stepSizeX)
        {
            double sn = std::sin(x * M_PI/2.5);
            mls.mergePoint(Eigen::Vector3d(x, y, cs*sn));
        }
    }

    GridMapF maxSteps,
             slopes;

    tools::MLSToSlopes mlsToSlopes;
    mlsToSlopes.computeMaxSteps(mls, maxSteps, true);
    mlsToSlopes.computeSlopes(mls, slopes);

    tools::SimpleTraversability simpleTraversability = tools::SimpleTraversability(0.8, 253, 1);

    TraversabilityGrid traversabilityGrid;
    simpleTraversability.calculateTraversability(traversabilityGrid,slopes, maxSteps);

    traversabilityGrid.getLocalFrame().translation() << 0.5 * traversabilityGrid.getSize(), 0;

    return traversabilityGrid;
}

static void showTraversabilityGrid(const TraversabilityGrid& traversabilityGrid, bool updateable = false)
{
    StandaloneVisualizer app;

    app.updateData(traversabilityGrid);
    std::cout << "update finished" << std::endl;
    Vector2ui numCells;

    while (app.wait(1000))
    {
        if (updateable)
        {
            std::cout << "Enter number of Cells (e.g.: 20 18):";
            while (!(std::cin >> numCells.x() >> numCells.y()))
            {
                std::cout << "Wrong input type!" << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
            app.updateData(sinesMlsToTravGrid(numCells, traversabilityGrid.getResolution()));
            std::cout << "update finished" << std::endl;
        }
    }
}

BOOST_AUTO_TEST_CASE(test_TraversabilityGridVisualisation)
{
    Vector2ui numCells(68, 125);
    Vector2d resolution(0.186, 0.1258);

    TraversabilityGrid traversabilityGrid = sinesMlsToTravGrid(numCells, resolution);
    showTraversabilityGrid(traversabilityGrid);
}

BOOST_AUTO_TEST_CASE(test_TraversabilityGridVisualisation_min)
{
    Vector2ui numCells(4, 4);
    Vector2d resolution(0.05, 0.05);

    TraversabilityGrid traversabilityGrid = sinesMlsToTravGrid(numCells, resolution);
    showTraversabilityGrid(traversabilityGrid);
}


BOOST_AUTO_TEST_CASE(test_TraversabilityGridVisualisation_update)
{
    Vector2ui numCells(59, 125);
    Vector2d resolution(0.186, 0.1258);

    TraversabilityGrid traversabilityGrid = sinesMlsToTravGrid(numCells, resolution);
    showTraversabilityGrid(traversabilityGrid, true);
}
