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

#include <maps/grid/MLSMap.hpp>
#include <maps/grid/TraversabilityGrid.hpp>
#include <maps/grid/GridMap.hpp>
#include <maps/grid/Index.hpp>

#include <maps/tools/MLSToSlopes.hpp>


using namespace maps;
using namespace grid;
using namespace tools;

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_flatPlane)
{
    const Vector2ui numCells(20, 20);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z = 5;
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), 0, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), 0, 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_45)
{
    const Vector2ui numCells(20, 20);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z = y * resolution[0];
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), M_PI/4, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), resolution[0], 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_Plane_withStdDev)
{
    const Vector2ui numCells(20, 20);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z = 5;
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();   
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps, true);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), 0, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), 2 * stddev, 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_45_withStdDev)
{
    const Vector2ui numCells(20, 30);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z = y * resolution[0];
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps, true);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), M_PI/4, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), resolution[0] + 2 * stddev, 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_Plane_windowSizeLimit)
{
    const Vector2ui numCells(5, 5);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z = 5;
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes, 5);
    MLSToSlopes::computeMaxSteps(mls, maxSteps, true);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), 0, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), 2 * stddev, 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_Plane_withStdDev_holesInMLS)
{
    const Vector2ui numCells(40, 40);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            if (x >= 10 && x <= 20 && y >= 15 && y <= 25)
                continue;
            float z = 5;
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps, true);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else if (x >= 10 && x <= 20 && y >= 15 && y <= 25)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), 0, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), 2 * stddev, 0.0001);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_MLSToSlopes_45_half_and_half)
{
    const Vector2ui numCells(40, 40);
    const Vector2d resolution(0.01, 0.01);
    MLSMapKalman mls = MLSMapKalman();
    mls.resize(numCells);
    mls.setResolution(resolution);
    float stddev = 0.1;
    float variance = stddev * stddev;
    
    for (uint x = 0; x < numCells[0] ; ++x)
    {
        for (uint y = 0; y < numCells[1]; ++y)
        {
            float z;
            if (y >= numCells[1] / 2)
                z = ((numCells[1] / 2) - y + numCells[1] / 2) * resolution[0];
            else z = y * resolution[0];
            
            MLSMapKalman::Patch newPatch = MLSMapKalman::Patch(z, variance);
            mls.mergePatch(grid::Index(x, y), newPatch);
        }
    }
    
    GridMapF slopes = GridMapF();
    GridMapF maxSteps = GridMapF();
    
    MLSToSlopes::computeSlopes(mls, slopes);
    MLSToSlopes::computeMaxSteps(mls, maxSteps, true);
    
    for (size_t y = 0; y < numCells[1]; ++y)
    {
        for (size_t x = 0; x < numCells[0]; ++x)
        {
            if (x <= 0 || x >= numCells[0] - 1 || y <= 0 || y >= numCells[1] - 1)
            {
                BOOST_CHECK_EQUAL(slopes.at(Index(x, y)), -std::numeric_limits<double>::infinity());
                BOOST_CHECK_EQUAL(maxSteps.at(Index(x, y)), -std::numeric_limits<double>::infinity());
            }
            else if (y == numCells[1] / 2)
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), 0, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), resolution[0] + 2 * stddev, 0.0001);
            }
            else
            {
                BOOST_CHECK_CLOSE(slopes.at(Index(x, y)), M_PI/4, 0.0001);
                BOOST_CHECK_CLOSE(maxSteps.at(Index(x, y)), resolution[0] + 2 * stddev, 0.0001);
            }
        }
    }
}