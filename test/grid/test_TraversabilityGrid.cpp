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
#define BOOST_TEST_MODULE GridTest
#include <boost/test/unit_test.hpp>

#include <maps/grid/TraversabilityGrid.hpp>

using namespace maps::grid;

struct Fixture
{
    Fixture()
    {
    }

    ~Fixture()
    {
    }

    // Product of numCells has to be kept under 256.
    // Otherwise Grid setter and getter test need to be adjusted.
    Vector2ui numCells = Vector2ui(10, 15);
    Vector2d resolution = Vector2d(0.134, 0.432);
    uint8_t default_traversabilityClassId = std::numeric_limits<uint8_t>::max();
    float default_probability = 0.9;
    uint8_t maxuint8_t = std::numeric_limits<uint8_t>::max();

    TraversabilityGrid traversabilityGrid = TraversabilityGrid(numCells, resolution, TraversabilityCell(default_traversabilityClassId, default_probability * maxuint8_t));
};

BOOST_AUTO_TEST_CASE(test_traversabilityClass_default_constructor)
{
    // Default constructor initializes with -1.
    // (getDrivability always returns >= 0)
    TraversabilityClass traversabilityClass = TraversabilityClass();
    BOOST_CHECK_EQUAL(traversabilityClass.getDrivability(), 0);
    BOOST_CHECK_EQUAL(traversabilityClass.isClassDefined(), false);
    BOOST_CHECK_EQUAL(traversabilityClass.isTraversable(), false);

}

BOOST_AUTO_TEST_CASE(test_traversabilityClass_parameterized_constructor)
{
    // Check TraversabilityClass parameterized constructor.
    TraversabilityClass traversabilityClass = TraversabilityClass(12);
    BOOST_CHECK_EQUAL(traversabilityClass.getDrivability(), 12);
    BOOST_CHECK_EQUAL(traversabilityClass.isClassDefined(), true);
    BOOST_CHECK_EQUAL(traversabilityClass.isTraversable(), true);
}

BOOST_AUTO_TEST_CASE(test_traversabilityCell_default_constructor)
{
    TraversabilityCell traversabilityCell = TraversabilityCell();
    BOOST_CHECK_EQUAL(traversabilityCell.getTraversabilityClassId(), 0);
    BOOST_CHECK_EQUAL(traversabilityCell.getProbability(), 0);
}

BOOST_AUTO_TEST_CASE(test_traversabilityCell_parameterized_constructor)
{
    TraversabilityCell traversabilityCell = TraversabilityCell(1, 2);
    BOOST_CHECK_EQUAL(traversabilityCell.getTraversabilityClassId(), 1);
    BOOST_CHECK_EQUAL(traversabilityCell.getProbability(), 2);
}

BOOST_AUTO_TEST_CASE(test_traversabilityCell_operators)
{
    TraversabilityCell traversabilityCell_1 = TraversabilityCell(14, 15);
    TraversabilityCell traversabilityCell_2 = TraversabilityCell(30, 31);

    BOOST_CHECK_EQUAL((traversabilityCell_1 == traversabilityCell_2), false);

    TraversabilityCell traversabilityCell_3 = traversabilityCell_1;

    BOOST_CHECK_EQUAL((traversabilityCell_3 != traversabilityCell_1), false);
}

BOOST_AUTO_TEST_CASE(test_traversabilityGrid_default_constructor)
{
    TraversabilityGrid traversabilityGrid = TraversabilityGrid();
    BOOST_CHECK_EQUAL(traversabilityGrid.getNumCells(), Vector2ui(0, 0));
}

BOOST_FIXTURE_TEST_CASE(test_traversabilityGrid_standard_constructor, Fixture)
{
    // Constructer is called in the Fixture.
    // Check for proper initilization with default values.
    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            BOOST_CHECK_CLOSE(traversabilityGrid.getProbability(x, y), default_probability, 1);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), default_traversabilityClassId);
        }
    }

    // Check state of traversabilityClasses (should not be initialized).
    BOOST_CHECK(traversabilityGrid.getTraversabilityClasses().size() == 0);
}

BOOST_FIXTURE_TEST_CASE(test_setTraversabilityClass, Fixture)
{
    TraversabilityClass traversabilityClass;

    for (unsigned int i = 0; i <= maxuint8_t; ++i)
    {
        traversabilityClass = TraversabilityClass(i + 1);
        traversabilityGrid.setTraversabilityClass(i, traversabilityClass);
        BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClass(i).getDrivability(), traversabilityClass.getDrivability());
        BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), i + 1);
    }
}

BOOST_FIXTURE_TEST_CASE(test_registerNewTraversabilityClass, Fixture)
{
    uint8_t returnId;
    TraversabilityClass traversabilityClass;

    for (unsigned int i = 0; i <= maxuint8_t; ++i)
    {
        traversabilityClass = TraversabilityClass(i + 1);
        traversabilityGrid.registerNewTraversabilityClass(returnId, traversabilityClass);
        BOOST_CHECK_EQUAL(returnId, i);
        BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClass(i).getDrivability(), traversabilityClass.getDrivability());
        BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), i + 1);
    }

    BOOST_CHECK(traversabilityGrid.registerNewTraversabilityClass(returnId, TraversabilityClass(300)) == false);
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), maxuint8_t + 1);
}

BOOST_FIXTURE_TEST_CASE(test_probability_setters_getters, Fixture)
{
    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            BOOST_CHECK(traversabilityGrid.setProbability(y / 100.0, x, y));
            BOOST_CHECK_CLOSE(traversabilityGrid.getProbability(x, y), static_cast<float>(static_cast<int>((y / 100.0) * maxuint8_t)) / maxuint8_t, 1);
        }
    }
    BOOST_CHECK_EQUAL(traversabilityGrid.setProbability(-1, 0, 0), false);
    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(0, 0), 0);

    BOOST_CHECK_EQUAL(traversabilityGrid.setProbability(1.1, 0, 0), false);
    BOOST_CHECK_EQUAL(traversabilityGrid.getProbability(0, 0), 0);
}

BOOST_FIXTURE_TEST_CASE(test_for_probability_drift, Fixture)
{
    // Check for probability drift.
    traversabilityGrid.setProbability(0.1 , 0, 0);
    for (uint i = 0; i < 10; ++i)
    {
        traversabilityGrid.setProbability(traversabilityGrid.getProbability(0, 0) , 0, 0);
        BOOST_CHECK_CLOSE(traversabilityGrid.getProbability(0, 0), static_cast<float>(static_cast<int>(0.1 * maxuint8_t)) / maxuint8_t, 1);
    }
}

BOOST_FIXTURE_TEST_CASE(test_traversability_setters_getters, Fixture)
{
    // Fill traversabilityClasses.
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 0);

    for (unsigned int i = 0; i <= maxuint8_t; ++i)
    {
        traversabilityGrid.setTraversabilityClass(i, TraversabilityClass(i + 1));
    }

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            traversabilityGrid.setTraversability(x + numCells[0] * y, x, y);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversability(x, y).getDrivability(), x + numCells[0] * y + 1);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), x + numCells[0] * y);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(test_setTraversabilityAndProbability, Fixture)
{
    // Fill traversabilityClasses.
    BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClasses().size(), 0);

    for (unsigned int i = 0; i <= maxuint8_t; ++i)
    {
        traversabilityGrid.setTraversabilityClass(i, TraversabilityClass(i + 1));
    }

    for (unsigned int y = 0; y < numCells[1]; ++y)
    {
        for (unsigned int x = 0; x < numCells[0]; ++x)
        {
            BOOST_CHECK(traversabilityGrid.setTraversabilityAndProbability(x + numCells[0] * y, y / 100.0, x, y));
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversability(x, y).getDrivability(), x + numCells[0] * y + 1);
            BOOST_CHECK_EQUAL(traversabilityGrid.getTraversabilityClassId(x, y), x + numCells[0] * y);
            BOOST_CHECK_CLOSE(traversabilityGrid.getProbability(x, y), static_cast<float>(static_cast<int>((y / 100.0) * maxuint8_t)) / maxuint8_t, 1);
        }
    }
    BOOST_CHECK_EQUAL(traversabilityGrid.setTraversabilityAndProbability(5, -0.1, 0, 0), false);
    BOOST_CHECK_EQUAL(traversabilityGrid.setTraversabilityAndProbability(0, 1.1, 0, 0), false);
}
