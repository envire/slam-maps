#include <boost/test/unit_test.hpp>
#include <envire_maps/GridBase.hpp>

#include <chrono>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_grid_index)
{
    GridBase::Index index;
    BOOST_CHECK_EQUAL(index.x, 0);
    BOOST_CHECK_EQUAL(index.y, 0);

    GridBase::Index index2(2, 4);
    BOOST_CHECK_EQUAL(index2.x, 2);
    BOOST_CHECK_EQUAL(index2.y, 4);         

    BOOST_CHECK_EQUAL((index2 < GridBase::Index(3, 5)), true);  
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(2, 5)), true);  
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(2, 4)), false);     
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(2, 3)), false); 
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(1, 5)), false);     
    BOOST_CHECK_EQUAL((index2 < GridBase::Index(1, 3)), false);     

    BOOST_CHECK_EQUAL((index2 == GridBase::Index(2, 4)), true);
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(3, 4)), false);    
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(2, 5)), false);    
    BOOST_CHECK_EQUAL((index2 == GridBase::Index(3, 5)), false);    

    BOOST_CHECK_EQUAL((index2 != GridBase::Index(2, 4)), false);
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(3, 4)), true); 
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(2, 5)), true); 
    BOOST_CHECK_EQUAL((index2 != GridBase::Index(3, 5)), true);     

    BOOST_CHECK((index2 + GridBase::Index(3, 5)) == GridBase::Index(5, 9));     

    // TODO: check otherwise => it will not be 3,5
    BOOST_CHECK((GridBase::Index(5, 9) - index2) == GridBase::Index(3, 5));     
}


BOOST_AUTO_TEST_CASE(test_gridbase)
{
    // the grid 100 x 200 (10m x 100m)
    size_t cellSizeX = 100;
    size_t cellSizeY = 200;

    // cell size 0.1m x 0.5m
    double scaleX = 0.1;
    double scaleY = 0.5;

    // the grid center is -5m x -50m
    double offsetX = -5;
    double offsetY = -50;

    GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);  

    GridBase grid_base(config);

    BOOST_CHECK_EQUAL(grid_base.getCellSizeX(), cellSizeX);
    BOOST_CHECK_EQUAL(grid_base.getCellSizeY(), cellSizeY);
    BOOST_CHECK_EQUAL(grid_base.getScaleX(), scaleX);
    BOOST_CHECK_EQUAL(grid_base.getScaleY(), scaleY);
    BOOST_CHECK_EQUAL(grid_base.getOffsetX(), offsetX);
    BOOST_CHECK_EQUAL(grid_base.getOffsetY(), offsetY); 
    BOOST_CHECK_EQUAL(grid_base.getSizeX(), 10);
    BOOST_CHECK_EQUAL(grid_base.getSizeY(), 100);

    // ---- Index 2 Position ---- 

    Eigen::Vector2d pos;

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(0, 0), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), -4.95, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), -49.75, 0.0001); 

    // top left
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(99, 199), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), 4.95, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 49.75, 0.0001);      

    // middle
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(50, 100), pos), true);
    BOOST_CHECK_CLOSE(pos.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos.y(), 0.25, 0.0001);

    // outside
    BOOST_CHECK_EQUAL(grid_base.fromGrid(GridBase::Index(100, 200), pos), false);   


    // ---- Position 2 Index ---- 

    GridBase::Index idx;
    Eigen::Vector2d pos_diff;

    // bottom right
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(-4.95, -49.75), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 0);
    BOOST_CHECK_EQUAL(idx.y, 0);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.25, 0.0001);

    // top left
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(4.95, 49.75), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 99);
    BOOST_CHECK_EQUAL(idx.y, 199);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.05, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.25, 0.0001);  

    // middle
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(0, 0), idx, pos_diff), true);
    BOOST_CHECK_EQUAL(idx.x, 50);
    BOOST_CHECK_EQUAL(idx.y, 100);
    BOOST_CHECK_CLOSE(pos_diff.x(), 0.0, 0.0001);
    BOOST_CHECK_CLOSE(pos_diff.y(), 0.0, 0.0001);   

    // outside
    BOOST_CHECK_EQUAL(grid_base.toGrid(Eigen::Vector2d(5, 50), idx, pos_diff), false);
}

