#include <boost/test/unit_test.hpp>
#include <envire_maps/GridMap.hpp>

#include <chrono>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_index)
{
    // check default constructor
    GridBase::Index index;
    BOOST_CHECK_EQUAL(index.x(), 0);
    BOOST_CHECK_EQUAL(index.y(), 0);

    // check constructor with parameter
    GridBase::Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x(), 2);
    BOOST_CHECK_EQUAL(index2.y(), 4);         

    // check the "<" operator
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x + 1, index2.y + 1)), true);  
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y)), false);       
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y + 1)), true);    
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x, index2.y - 1)), false); 
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x - 1, index2.y + 1)), false);     
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(index2.x - 1, index2.y - 1)), false);     

    // check the "==" operator
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x, index2.y)), true);
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x, index2.y + 1)), false);        
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x + 1, index2.y)), false);    
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(index2.x + 1, index2.y + 1)), false);    

    // check the "!=" operator
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x, index2.y)), false);
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x, index2.y + 1)), true);     
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x + 1, index2.y)), true); 
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(index2.x + 1, index2.y + 1)), true);     

    // check the sum operator
    BOOST_CHECK((index2 + GridBase::Index(3, 5)) == GridBase::Index(5, 9));     

    // TODO: check otherwise => it will not be 3,5
    // check the diff operator
    BOOST_CHECK((GridBase::Index(5, 9) - index2) == GridBase::Index(3, 5));     
}

// the grid 100 x 200 cells
Eigen::Vector2ui num_cells(100, 200);

// resolution 0.1m x 0.5m (therefore size will be 10 x 100m)
Eigen::Vector2d resolution (0.1, 0.5);

BOOST_AUTO_TEST_CASE(test_gridbase_default_constructor)
{
    GridBase grid_base;
    BOOST_CHECK_EQUAL(grid_base.getNumCells(), Eigen::Vector2ui::Zero());
    BOOST_CHECK_EQUAL(grid_base.getResolution(), Eigen::Vector2d::Zero());
    BOOST_CHECK_EQUAL(grid_base.getSize(), Eigen::Vector2d::Zero());

    BOOST_CHECK_EQUAL(grid_base.inGrid(GridBase::Index(0, 0)), false);

    Eigen::Vector2d pos;
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(0, 0), pos), false);

    GridBase::Index idx;
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(0, 0), idx), false);

}


