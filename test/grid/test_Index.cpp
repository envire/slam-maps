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

// This warning is slightly annoying (and likely only occurs because all values are known at compile time here):
#pragma GCC diagnostic ignored "-Wstrict-overflow"

#include <maps/grid/Index.hpp>
#include <iostream>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(test_index)
{
    // Index is (x,y)

    // check default constructor
    Index index;
    BOOST_CHECK_EQUAL(index.x(), 0);
    BOOST_CHECK_EQUAL(index.y(), 0);

    // check constructor with parameter
    Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x(), 2);
    BOOST_CHECK_EQUAL(index2.y(), 4);

    // Check constructor using Eigen expressions
    Index index3(Eigen::Matrix<int, 2, 1>(3, 4));
    BOOST_CHECK_EQUAL(index3.x(), 3);
    BOOST_CHECK_EQUAL(index3.y(), 4);

    // check the "<" operator: lexicographical ordering
    // two cases should return true: (x is smaller) or (x is equal and y is smaller)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() + 1, index2.y() + 1)), true);  //(2,4) < (3,5)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() + 1)), true);      //(2,4) < (2,5)  

    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y())), false);         //(2,4) < (2,4)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x(), index2.y() - 1)), false);     //(2,4) < (2,3)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() + 1)), false); //(2,4) < (1,5)
    BOOST_CHECK_EQUAL((index2 < Index(index2.x() - 1, index2.y() - 1)), false); //(2,4) < (1,3)
    BOOST_CHECK_EQUAL((index2 < Index(0,0)), false);                            //(2,4) < (0,0)

    // check the isInside: x and y are smaller
    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x() + 1, index2.y() + 1))), true);  //(2,4) < (3,5)

    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x(), index2.y() + 1))), false);     //(2,4) < (2,5)  
    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x(), index2.y()))), false);         //(2,4) < (2,4)
    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x(), index2.y() - 1))), false);     //(2,4) < (2,3)
    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x() - 1, index2.y() + 1))), false); //(2,4) < (1,5)
    BOOST_CHECK_EQUAL((index2.isInside(Index(index2.x() - 1, index2.y() - 1))), false); //(2,4) < (1,3)
    BOOST_CHECK_EQUAL((index2.isInside(Index(0,0))), false);                            //(2,4) < (0,0)   

    // check the "==" operator
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y())), true);         //(2,4) == (2,4)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x(), index2.y() + 1)), false);    //(2,4) == (2,5)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y())), false);    //(2,4) == (3,4)
    BOOST_CHECK_EQUAL((index2 == Index(index2.x() + 1, index2.y() + 1)), false);//(2,4) == (3,5)    

    // check the "!=" operator
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y())), false);        //(2,4) == (2,4)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x(), index2.y() + 1)), true);     //(2,4) == (2,5)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y())), true);     //(2,4) == (3,4)
    BOOST_CHECK_EQUAL((index2 != Index(index2.x() + 1, index2.y() + 1)), true); //(2,4) == (3,5)

    // check the sum operator
    BOOST_CHECK((index2 + Index(3, 5)) == Index(5, 9));     //(2,4) + (3,5) = (5,9)

    // check the sum operator
    BOOST_CHECK((index2 + Index(-3, -5)) == Index(-1, -1)); //(2,4) + (-3, -5) = (-1,-1)

    // check the subtract operator
    BOOST_CHECK((Index(5, 9) - index2) == Index(3, 5));     //(5,9) - (2,4) = (3,5)

    // check the subtract operator
    BOOST_CHECK((index2 - Index(5, 9)) == Index(-3, -5));   //(2,4) - (5,9) = (-3, -5)    
}
