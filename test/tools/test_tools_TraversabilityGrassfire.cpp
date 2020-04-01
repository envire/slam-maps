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

#include <maps/grid/GridMap.hpp>
#include <maps/grid/MLSMap.hpp>
#include <maps/grid/TraversabilityGrid.hpp>
#include <maps/tools/TraversabilityGrassfire.hpp>

#include <maps/tools/SimpleTraversability.hpp>

using namespace maps;
using namespace tools;

struct Fixture
{
    Fixture()
    {
    }

    ~Fixture()
    {
    }

    const double maxStepHeight = 0.15,
                 maxSlope = 0.8,
                 robotHeight = 1.2,
                 nominalStdDev = 0.1,
                 outliertFilterMaxStdDev = 1.0;

    const int numTraversabilityClasses = 10;

    // Used for small maps.
    grid::Vector2ui numCellsMin = grid::Vector2ui(4, 4);
    grid::Vector2d resolutionMin = grid::Vector2d(0.1, 0.1);
    Eigen::Vector3d startPosMin = Eigen::Vector3d(0.05, 0.05, 0.0);
    grid::MLSMapKalman mlsMin = grid::MLSMapKalman(numCellsMin, resolutionMin, mlsMin.getConfig());

    // Used for bigger maps.
    grid::Vector2ui numCells = grid::Vector2ui(20, 20);
    grid::Vector2d resolution = grid::Vector2d(0.1, 0.1);
    Eigen::Vector3d startPos = Eigen::Vector3d(0.95, 0.95, 0.0);
    grid::MLSMapKalman mls = grid::MLSMapKalman(numCells, resolution, mls.getConfig());

    TraversabilityGrassfireConfig config = TraversabilityGrassfireConfig(maxStepHeight,
                                                                         maxSlope,
                                                                         robotHeight,
                                                                         numTraversabilityClasses,
                                                                         nominalStdDev,
                                                                         outliertFilterMaxStdDev);

    grid::TraversabilityGrid traversabilityGrid = grid::TraversabilityGrid();
    TraversabilityGrassfire traversabilityGrassfire = TraversabilityGrassfire(config);

    void addWalls(grid::MLSMapKalman& mls, grid::Vector2ui& start, grid::Vector2ui& end, double mean, double height = 0)
    {
        for (size_t y = 0; y < numCells.y(); ++y)
        {
            for (size_t x = 0; x < numCells.x(); ++x)
            {
                if (x == start.x() || x == end.x() || y == start.y() || y == end.y())
                    mls.mergePatch(grid::Index(x, y), grid::SurfacePatch<grid::MLSConfig::KALMAN>(mean, 0.01, height));
                else
                    mls.mergePatch(grid::Index(x, y), grid::SurfacePatch<grid::MLSConfig::KALMAN>(0.0, 0.01));;
            }
        }
    }
};

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_min_constSlope, Fixture)
{
    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            mlsMin.mergePoint(Eigen::Vector3d(x * 0.1 + 0.05, y * 0.1 + 0.05, y * 0.04));
        }
    }
    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mlsMin, startPosMin);

    BOOST_CHECK_EQUAL(traversabilityGrid.getNumCells(), mlsMin.getNumCells());
    BOOST_CHECK_EQUAL(traversabilityGrid.getResolution(), mlsMin.getResolution());
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), numTraversabilityClasses + 2);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            if (!((x == 0 || x == numCellsMin.x() - 1) && (y == 0 || y == numCellsMin.y() - 1)))
            {
                double slopeInRad = (1 - atan2(0.4, 1));
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1.0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + static_cast<int>(slopeInRad * 10));
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), static_cast<int>(slopeInRad * 10) / 10., 0.1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_min_empty, Fixture)
{
    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mlsMin, startPosMin);

    BOOST_CHECK_EQUAL(traversabilityGrid.getNumCells(), mlsMin.getNumCells());
    BOOST_CHECK_EQUAL(traversabilityGrid.getResolution(), mlsMin.getResolution());
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), numTraversabilityClasses + 2);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_min_obstacleFilled_slope, Fixture)
{
    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            mlsMin.mergePoint(Eigen::Vector3d(x * 0.1 + 0.05, y * 0.1 + 0.05, y * 0.12));
        }
    }

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mlsMin, startPosMin);

    BOOST_CHECK_EQUAL(traversabilityGrid.getNumCells(), mlsMin.getNumCells());
    BOOST_CHECK_EQUAL(traversabilityGrid.getResolution(), mlsMin.getResolution());
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), numTraversabilityClasses + 2);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            if (!((x == 0 || x == numCellsMin.x() - 1) && (y == 0 || y == numCellsMin.y() - 1)))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1.0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 0, 0.1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_min_outlier, Fixture)
{
    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            mlsMin.mergePoint(Eigen::Vector3d(x * 0.1 + 0.05, y * 0.1 + 0.05, 0));
        }
    }

    mlsMin.mergePatch(grid::Index(2, 2), grid::SurfacePatch<grid::MLSConfig::KALMAN>(1.1, 36));

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mlsMin, startPosMin);

    BOOST_CHECK_EQUAL(traversabilityGrid.getNumCells(), mlsMin.getNumCells());
    BOOST_CHECK_EQUAL(traversabilityGrid.getResolution(), mlsMin.getResolution());
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), numTraversabilityClasses + 2);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCellsMin.y(); ++y)
    {
        for (size_t x = 0; x < numCellsMin.x(); ++x)
        {
            if (!((x == 0 || x == numCellsMin.x() - 1) && (y == 0 || y == numCellsMin.y() - 1)))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + numTraversabilityClasses);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_multiLevel, Fixture)
{
    for (size_t y = 0; y < numCells.y(); ++y)
    {
        for (size_t x = 0; x < numCells.x(); ++x)
        {
            mls.mergePoint(Eigen::Vector3d(x * 0.1 + 0.05, y * 0.1 + 0.05, 0.0));
        }
    }

    mls.mergePatch(grid::Index(2, 2), grid::SurfacePatch<grid::MLSConfig::KALMAN>(2.0, 0.01));
    mls.mergePatch(grid::Index(4, 4), grid::SurfacePatch<grid::MLSConfig::KALMAN>(1.1, 0.01));

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mls, startPosMin);

    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 2 + numTraversabilityClasses);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCells.y(); ++y)
    {
        for (size_t x = 0; x < numCells.x(); ++x)
        {
            if (!((x == 0 || x == numCells.x() - 1) && (y == 0 || y == numCells.y() - 1)))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1.0);

                if (!(x > 2 && x < 6 && y > 2 && y < 6))
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + numTraversabilityClasses);
                else
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_wall_low, Fixture)
{
    grid::Vector2ui start = grid::Vector2ui(2, 2),
                end = grid::Vector2ui(mls.getNumCells().x() - 3, mls.getNumCells().y() - 3);

    addWalls(mls, start, end, 0.2);

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mls, startPos);

    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 2 + numTraversabilityClasses);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCells.y(); ++y)
    {
        for (size_t x = 0; x < numCells.x(); ++x)
        {
            if (!((x == 0 || x == numCells.x() - 1) && (y == 0 || y == numCells.y() - 1)))
            {
                if ((((x >= start.x() && x <= start.x() + 1) || (x >= end.x() - 1  && x <= end.x())) && y >= start.y() && y <= end.y())
                    || (((y >= start.y() && y <= start.y() + 1) || (y >= end.y() - 1 && y <= end.y())) && x >= start.x() && x <= end.x()))
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
                }
                else if ((x > start.x() + 1) && (x < end.x() - 1) && (y > start.y() + 1) && (y < end.y() - 1))
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + numTraversabilityClasses);
                }
                else
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                }
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_wall_high, Fixture)
{
    grid::Vector2ui start = grid::Vector2ui(2, 2),
                end = grid::Vector2ui(mls.getNumCells().x() - 3, mls.getNumCells().y() - 3);

    addWalls(mls, start, end, 2.0);

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mls, startPos);

    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 2 + numTraversabilityClasses);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCells.y(); ++y)
    {
        for (size_t x = 0; x < numCells.x(); ++x)
        {
            if ((x == start.x() + 1 || x == end.x() - 1) && (y == start.y() + 1 || y == end.y() - 1))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
            }
            else if ((x > start.x()) && (x < end.x()) && (y > start.y()) && (y < end.y()))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + numTraversabilityClasses);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
            }
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_trav_grassfire_wall_high_vertical, Fixture)
{
    grid::Vector2ui start = grid::Vector2ui(2, 2),
                end = grid::Vector2ui(mls.getNumCells().x() - 3, mls.getNumCells().y() - 3);

    addWalls(mls, start, end, 2.0, 2.0);

    traversabilityGrassfire.calculateTraversability(traversabilityGrid, mls, startPos);

    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 2 + numTraversabilityClasses);

    for (size_t i = 2; i < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i).getDrivability(), 1.0 * (i - 1) / numTraversabilityClasses, 0.1);
    }

    for (size_t y = 0; y < numCells.y(); ++y)
    {
        for (size_t x = 0; x < numCells.x(); ++x)
        {
            if (!((x == 0 || x == numCells.x() - 1) && (y == 0 || y == numCells.y() - 1)))
            {
                if ((((x >= start.x() && x <= start.x() + 1) || (x >= end.x() - 1  && x <= end.x())) && y >= start.y() && y <= end.y())
                    || (((y >= start.y() && y <= start.y() + 1) || (y >= end.y() - 1 && y <= end.y())) && x >= start.x() && x <= end.x()))
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
                }
                else if ((x > start.x() + 1) && (x < end.x() - 1) && (y > start.y() + 1) && (y < end.y() - 1))
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 1);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1 + numTraversabilityClasses);
                }
                else
                {
                    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                }
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(x, y), 0);
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 1., 0.1);
            }
        }
    }
}
