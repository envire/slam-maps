#include <boost/test/unit_test.hpp>
#include <envire_maps/Grid.hpp>
#include <iostream>

using namespace envire::maps;

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
    Index index3(Eigen::Matrix<unsigned int, 2, 1>(3, 4));
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

    // check the subtract operator
    BOOST_CHECK((Index(5, 9) - index2) == Index(3, 5));     //(5,9) - (2,4) = (3,5)
}