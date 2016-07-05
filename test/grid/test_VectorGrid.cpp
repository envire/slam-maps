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
