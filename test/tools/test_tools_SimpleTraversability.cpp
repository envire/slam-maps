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

#include "maps/grid/TraversabilityGrid.hpp"
#include "maps/grid/GridMap.hpp"
#include "maps/tools/SimpleTraversability.hpp"
#include "maps/grid/Index.hpp"
#include "typeinfo"

using namespace maps;
using namespace grid;
using namespace tools;

#include <boost/core/demangle.hpp>

struct Fixture
{
    Fixture()
    {
    }

    ~Fixture()
    {
    }

    const double maximumSlope = 0.5,
                minPassageWidth = 0.4829381,
                groundClearance = 3,
                obstacleClearance = 0.53123124;

    const int classCount = 20;

    const Vector2ui numCells = Vector2ui(50, 60);
    const Vector2d resolution = Vector2d(0.12354498, 0.241387654);

    SimpleTraversabilityConfig config =  SimpleTraversabilityConfig(maximumSlope, classCount, groundClearance, minPassageWidth, obstacleClearance);
    SimpleTraversabilityConfig confNoPP = SimpleTraversabilityConfig(maximumSlope, classCount, groundClearance);

    TraversabilityGrid traversabilityGrid = TraversabilityGrid();
    SimpleTraversability simpleTraversability = SimpleTraversability(config);

    GridMapF slopesIn = GridMapF(numCells, resolution, -std::numeric_limits<double>::infinity()),
             maxStepsIn = GridMapF(numCells, resolution, -std::numeric_limits<double>::infinity());

    void addSinuses(GridMapF& map, double amplitude, double wavelength, int direction)
    {
        for (unsigned int y = 0; y < numCells[1]; ++y)
        {
            for (unsigned int x = 0; x < numCells[0]; ++x)
            {
                if (map.at(x, y) == -std::numeric_limits<double>::infinity())
                    map.at(x, y) = 0;
                if (direction == 0)
                    map.at(x, y) += std::sin(x / (wavelength * 2)) * amplitude;
                if (direction == 1)
                    map.at(x, y) += std::sin(y / (wavelength * 2)) * amplitude;
            }
        }
    }
};

// CONSTRUCTORS
BOOST_AUTO_TEST_CASE(test_simpleTraversabilityConfig_defaultConstructor)
{
    SimpleTraversabilityConfig config = SimpleTraversabilityConfig();

    BOOST_CHECK_EQUAL(config.maximumSlope, 0);
    BOOST_CHECK_EQUAL(config.classCount, 0);
    BOOST_CHECK_EQUAL(config.groundClearance, 0);
    BOOST_CHECK_EQUAL(config.minPassageWidth, 0);
    BOOST_CHECK_EQUAL(config.obstacleClearance, 0);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversabilityConfig_parameterizedConstructor, Fixture)
{
    BOOST_CHECK_EQUAL(config.maximumSlope, maximumSlope);
    BOOST_CHECK_EQUAL(config.classCount, classCount);
    BOOST_CHECK_EQUAL(config.groundClearance, groundClearance);
    BOOST_CHECK_EQUAL(config.minPassageWidth, minPassageWidth);
    BOOST_CHECK_EQUAL(config.obstacleClearance, obstacleClearance);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversabilityConfig_parameterizedConstructor_noPostProcessing, Fixture)
{
    BOOST_CHECK_EQUAL(confNoPP.maximumSlope, maximumSlope);
    BOOST_CHECK_EQUAL(confNoPP.classCount, classCount);
    BOOST_CHECK_EQUAL(confNoPP.groundClearance, groundClearance);
    BOOST_CHECK_EQUAL(confNoPP.minPassageWidth, 0);
    BOOST_CHECK_EQUAL(confNoPP.obstacleClearance, 0);
}

BOOST_AUTO_TEST_CASE(test_simpleTraversability_defaultConstructor)
{
    SimpleTraversability simpleTraversability = SimpleTraversability();
    SimpleTraversabilityConfig confOut = simpleTraversability.getConfig();

    BOOST_CHECK_EQUAL(confOut.maximumSlope, 0);
    BOOST_CHECK_EQUAL(confOut.classCount, 0);
    BOOST_CHECK_EQUAL(confOut.groundClearance, 0);
    BOOST_CHECK_EQUAL(confOut.minPassageWidth, 0);
    BOOST_CHECK_EQUAL(confOut.obstacleClearance, 0);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_constructor_passedConfig, Fixture)
{
    SimpleTraversabilityConfig confOut = simpleTraversability.getConfig();

    BOOST_CHECK_EQUAL(confOut.maximumSlope, maximumSlope);
    BOOST_CHECK_EQUAL(confOut.classCount, classCount);
    BOOST_CHECK_EQUAL(confOut.groundClearance, groundClearance);
    BOOST_CHECK_EQUAL(confOut.minPassageWidth, minPassageWidth);
    BOOST_CHECK_EQUAL(confOut.obstacleClearance, obstacleClearance);
}

BOOST_AUTO_TEST_CASE(test_simpleTraversability_constructor_passedParamerters)
{
    SimpleTraversability simpleTraversability = SimpleTraversability(0, 1, 2, 3, 4);

    SimpleTraversabilityConfig confOut = simpleTraversability.getConfig();
    BOOST_CHECK_EQUAL(confOut.maximumSlope, 0);
    BOOST_CHECK_EQUAL(confOut.classCount, 1);
    BOOST_CHECK_EQUAL(confOut.groundClearance, 2);
    BOOST_CHECK_EQUAL(confOut.minPassageWidth, 3);
    BOOST_CHECK_EQUAL(confOut.obstacleClearance, 4);
}

BOOST_AUTO_TEST_CASE(test_simpleTraversability_constructor_passedParamerters_noPostProcessing)
{
    SimpleTraversability simpleTraversability = SimpleTraversability(0, 1, 2);

    SimpleTraversabilityConfig confOut = simpleTraversability.getConfig();
    BOOST_CHECK_EQUAL(confOut.maximumSlope, 0);
    BOOST_CHECK_EQUAL(confOut.classCount, 1);
    BOOST_CHECK_EQUAL(confOut.groundClearance, 2);
    BOOST_CHECK_EQUAL(confOut.minPassageWidth, 0);
    BOOST_CHECK_EQUAL(confOut.obstacleClearance, 0);
}

// SETTERS AND GETTERS
BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_settersAndGetters, Fixture)
{
    SimpleTraversabilityConfig config = simpleTraversability.getConfig();
    BOOST_CHECK_EQUAL(config.maximumSlope, maximumSlope);
    BOOST_CHECK_EQUAL(config.classCount, classCount);
    BOOST_CHECK_EQUAL(config.groundClearance, groundClearance);
    BOOST_CHECK_EQUAL(config.minPassageWidth, minPassageWidth);
    BOOST_CHECK_EQUAL(config.obstacleClearance, obstacleClearance);

    simpleTraversability.setConfig(0.1, 5, 0.2, 0.3, 0.4);

    config = simpleTraversability.getConfig();
    BOOST_CHECK_EQUAL(config.maximumSlope, 0.1);
    BOOST_CHECK_EQUAL(config.classCount, 5);
    BOOST_CHECK_EQUAL(config.groundClearance, 0.2);
    BOOST_CHECK_EQUAL(config.minPassageWidth, 0.3);
    BOOST_CHECK_EQUAL(config.obstacleClearance, 0.4);

    SimpleTraversabilityConfig newConf = SimpleTraversabilityConfig(1.1, 10, 1.2, 1.3, 1.4);
    simpleTraversability.setConfig(newConf);

    config = simpleTraversability.getConfig();
    BOOST_CHECK_EQUAL(config.maximumSlope, 1.1);
    BOOST_CHECK_EQUAL(config.classCount, 10);
    BOOST_CHECK_EQUAL(config.groundClearance, 1.2);
    BOOST_CHECK_EQUAL(config.minPassageWidth, 1.3);
    BOOST_CHECK_EQUAL(config.obstacleClearance, 1.4);

}

// CALCULATE_TRAVERSABILITY
BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_emptyMap, Fixture)
{
    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
            BOOST_CHECK_CLOSE(traversabilityGrid.getTraversability(x, y).getDrivability(), 0.55, 0.1);
        }
    }

    BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), 0.55, 0.1);
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClass(1).getDrivability(), 0);
    BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(SimpleTraversability::CUSTOM_CLASSES + classCount - 1).getDrivability(), 1, 0.1);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_obstacleFilled, Fixture)
{
    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            slopesIn.at(x, y) = 100;
        }
    }

    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversability(x, y).getDrivability(), 0);
        }
    }

    BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), 0.55, 0.1);
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClass(1).getDrivability(), 0);
    BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(SimpleTraversability::CUSTOM_CLASSES + classCount - 1).getDrivability(), 1, 0.1);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_slopeSinuses_horizontalAndVertical_noPostProcessing, Fixture)
{
    addSinuses(slopesIn, maximumSlope * 1.5, 50 * (resolution[0] / 1.5), 0);
    addSinuses(slopesIn, maximumSlope * 1.5, 50 * (resolution[1] / 1.5), 1);

    simpleTraversability.setConfig(confNoPP);
    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int i = 0; i - 2 < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i + 2).getDrivability(), 1.0 / classCount * (i + 1), 0.1);
    }

    int counter = 0;
    double idSum = 0;

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            if (traversabilityGrid.getTraversabilityClassId(x, y) != 0)
            {
                ++counter;
                idSum += traversabilityGrid.getTraversabilityClassId(x, y);
            }

            if (fabs(slopesIn.at(x, y)) > maximumSlope)
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), (classCount - 1) - rint(fabs(slopesIn.at(x, y)) / maximumSlope * (classCount - 1)) + 2);
            }
        }
    }

    int medianId = rint(idSum / counter);
    if (medianId > 1)
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), traversabilityGrid.getTraversabilityClass(medianId).getDrivability(), 0.1);
    else
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), 0.55, 0.1);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_maxStepSinuses_horizontalAndVertical_noPostProcessing, Fixture)
{
    addSinuses(maxStepsIn, groundClearance * 1.5, 50 * (resolution[0] / 1.5), 0);
    addSinuses(maxStepsIn, groundClearance * 1.5, 50 * (resolution[1] / 1.5), 1);

    simpleTraversability.setConfig(confNoPP);
    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int i = 0; i - 2 < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i + 2).getDrivability(), 1.0 / classCount * (i + 1), 0.1);
    }

    int counter = 0;
    double idSum = 0;

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            if (traversabilityGrid.getTraversabilityClassId(x, y) != 0)
            {
                ++counter;
                idSum += traversabilityGrid.getTraversabilityClassId(x, y);
            }

            if (fabs(maxStepsIn.at(x, y)) > groundClearance)
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
            }
        }
    }

    int medianId = rint(idSum / counter);
    if (medianId > 1)
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), traversabilityGrid.getTraversabilityClass(medianId).getDrivability(), 0.1);
    else
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), 0.55, 0.1);
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_maxStepAndSlopeSinuses_horizontalAndVertical_noPostProcessing, Fixture)
{
    addSinuses(maxStepsIn, groundClearance * 1.5, 50 * (resolution[0] / 1.5), 0);
    addSinuses(maxStepsIn, groundClearance * 1.5, 50 * (resolution[1] / 1.5), 1);
    addSinuses(slopesIn, maximumSlope * 1.5, 50 * (resolution[0] / 1.5), 0);
    addSinuses(slopesIn, maximumSlope * 1.5, 50 * (resolution[1] / 1.5), 1);

    simpleTraversability.setConfig(confNoPP);
    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int i = 0; i - 2 < traversabilityGrid.getTraversabilityClasses().size(); ++i)
    {
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(i + 2).getDrivability(), 1.0 / classCount * (i + 1), 0.1);
    }

    int counter = 0;
    double idSum = 0;

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            if (traversabilityGrid.getTraversabilityClassId(x, y) != 0)
            {
                ++counter;
                idSum += traversabilityGrid.getTraversabilityClassId(x, y);
            }

            if (fabs(maxStepsIn.at(x, y)) > groundClearance)
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
                continue;
            }

            if (fabs(slopesIn.at(x, y)) > maximumSlope)
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 1);
            }
            else if (!slopesIn.isDefault(slopesIn.at(x, y)))
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), (classCount - 1) - rint(fabs(slopesIn.at(x, y)) / maximumSlope * (classCount - 1)) + 2);
            }
            else
            {
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), 0);
                continue;
            }
        }
    }

    int medianId = rint(idSum / counter);
    if (medianId > 1)
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), traversabilityGrid.getTraversabilityClass(medianId).getDrivability(), 0.1);
    else
        BOOST_CHECK_CLOSE(traversabilityGrid.getTraversabilityClass(0).getDrivability(), 0.55, 0.1);
}


// POSTPROCESSING
BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_closeNarrowPassage, Fixture)
{
    unsigned int xPosition = numCells[0] / 2;
    unsigned int yPosition1 = numCells[1] / 3;
    unsigned int yPosition2 = minPassageWidth / resolution[1] + yPosition1;

    for (int x = -2; x <= 2; ++x)
    {
        unsigned int mapX = xPosition + x;
        if (mapX < 0 || yPosition1 < 0 || yPosition2 < 0 || mapX >= numCells[0] || yPosition1 >= numCells[1] || yPosition2 >= numCells[1])
            continue;
        maxStepsIn.at(xPosition + x, yPosition1) = groundClearance + 1;
        maxStepsIn.at(xPosition + x, yPosition2) = groundClearance + 1;
    }

    simpleTraversability.setConfig(config);

    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    for (unsigned int y = yPosition1; y <= yPosition2; ++y)
    {
        for (unsigned int x = xPosition - 2; x <= xPosition + 2; ++x)
        {
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(xPosition, y), 1);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_simpleTraversability_growObstacles, Fixture)
{
    int xPosition = numCells[0] / 2;
    int yPosition = numCells[1] / 2;

    maxStepsIn.at(xPosition, yPosition) = groundClearance + 1;

    simpleTraversability.setConfig(config);
    simpleTraversability.calculateTraversability(traversabilityGrid, slopesIn, maxStepsIn);

    int growthWidthX = obstacleClearance / resolution[0];
    int growthWidthY = obstacleClearance / resolution[1];

    for (int y = -growthWidthY; y <= growthWidthY; ++y)
    {
        for (int x = -growthWidthX; x <= growthWidthX; ++x)
        {
            unsigned int mapX = xPosition + x;
            unsigned int mapY = yPosition + y;
            if (mapX < 0 || mapY < 0 || mapX >= numCells[0] || mapY >= numCells[1])
                continue;

            double squaredDistance = pow(x * resolution[0], 2) + pow(y * resolution[1], 2);

            if (squaredDistance < pow(obstacleClearance, 2))
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(mapX, mapY), 1);
            else
                BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(mapX, mapY), 0);
        }
    }
}
