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

#include <maps/grid/VectorGrid.hpp>

using namespace ::maps::grid;

template <typename CellT>
class VectorGridTest : public VectorGrid<CellT>
{
public:
    using VectorGrid<CellT>::VectorGrid;
    
    std::pair<typename VectorGrid<CellT>::const_iterator, typename VectorGrid<CellT>::const_iterator> getRangeTest()
    {
        return this->getRange();
    }
};

BOOST_AUTO_TEST_CASE(test_get_range)
{
    VectorGridTest<double> vector_grid(Vector2ui(2, 3), -5);

    std::pair<VectorGridTest<double>::const_iterator, VectorGridTest<double>::const_iterator> range;

    // No elements
    range = vector_grid.getRangeTest();

    BOOST_CHECK(range.first == vector_grid.end());
    BOOST_CHECK(range.second == vector_grid.end());

    // First element 
    vector_grid.at(0, 0) = 0;

    range = vector_grid.getRangeTest();

    BOOST_CHECK_EQUAL(*range.first, 0);
    BOOST_CHECK_EQUAL(*(range.second-1), 0);
    BOOST_CHECK(range.first == vector_grid.begin());
    BOOST_CHECK(range.second == vector_grid.begin() + 1);

    // Last element
    vector_grid.at(0, 0) = -5;
    vector_grid.at(1, 2) = 5;

    range = vector_grid.getRangeTest();

    BOOST_CHECK_EQUAL(*range.first, 5);
    BOOST_CHECK_EQUAL(*(range.second-1), 5);
    BOOST_CHECK(range.first == (vector_grid.end() - 1));
    BOOST_CHECK(range.second == vector_grid.end());

    // First and last element
    vector_grid.at(0, 0) = 0;

    range = vector_grid.getRangeTest();

    BOOST_CHECK_EQUAL(*range.first, 0);
    BOOST_CHECK_EQUAL(*(range.second-1), 5);
    BOOST_CHECK(range.first == vector_grid.begin());
    BOOST_CHECK(range.second == vector_grid.end());

    // In the middle
    vector_grid.at(0, 0) = -5;
    vector_grid.at(1, 2) = -5;

    vector_grid.at(1, 0) = 1;
    vector_grid.at(0, 2) = 4;

    range = vector_grid.getRangeTest();

    BOOST_CHECK_EQUAL(*range.first, 1);
    BOOST_CHECK_EQUAL(*(range.second-1), 4);
    BOOST_CHECK(range.first == (vector_grid.begin() + 1));
    BOOST_CHECK(range.second == (vector_grid.end() - 1)); 
}
