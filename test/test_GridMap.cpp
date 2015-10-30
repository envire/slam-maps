    #include <boost/test/unit_test.hpp>
#include <envire_maps/GridMap.hpp>

using namespace envire::maps;
// the grid 20m x 50m
size_t cellSizeX = 100;
size_t cellSizeY = 100;

// cell size 0.2m x 0.5m
double scaleX = 0.2;
double scaleY = 0.5;

// the grid center is 20m x 50m
double offsetX = 10;
double offsetY = 25;

GridConfig config(cellSizeX, cellSizeY, scaleX, scaleY, offsetX, offsetY);

BOOST_AUTO_TEST_CASE(test_gridmap_has)
{   
    GridMap grid_map(config);

    // check if the not existed grid exists: false
    BOOST_CHECK_EQUAL(grid_map.hasGrid("double_grid"), false);

    // check if the existed grid exists: true
    grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid_map.hasGrid("double_grid"), true);

    // check if the existence of the grid after it was removed
    grid_map.removeGrid("double_grid"); 
    BOOST_CHECK_EQUAL(grid_map.hasGrid("double_grid"), false);  
}

BOOST_AUTO_TEST_CASE(test_gridmap_add)
{   
    GridMap grid_map(config);

    // add grid that does not exist until now: OK
    BOOST_CHECK_NO_THROW(grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity())); 

    // add grid with the key existed already and of the same type: THROW
    BOOST_CHECK_THROW(grid_map.addGrid<double>("double_grid",  std::numeric_limits<double>::infinity()), std::exception);

    // add grid with the key existed already but of the different type: THROW
    BOOST_CHECK_THROW(grid_map.addGrid<int>("double_grid",  std::numeric_limits<int>::infinity()), std::exception);

    // add grid with the key not existed and of the different type: true
    BOOST_CHECK_NO_THROW(grid_map.addGrid<int>("int_grid",  std::numeric_limits<int>::infinity()));     
}

BOOST_AUTO_TEST_CASE(test_gridmap_config)
{   
    GridMap grid_map(config);

    Grid<double> &grid = grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());  

    BOOST_CHECK_EQUAL(grid_map.getCellSizeX(), grid.getCellSizeX());
    BOOST_CHECK_EQUAL(grid_map.getCellSizeY(), grid.getCellSizeY());
    BOOST_CHECK_EQUAL(grid_map.getScaleX(), grid.getScaleX());  
    BOOST_CHECK_EQUAL(grid_map.getScaleY(), grid.getScaleY());      
    BOOST_CHECK_EQUAL(grid_map.getOffsetX(), grid.getOffsetX());    
    BOOST_CHECK_EQUAL(grid_map.getOffsetY(), grid.getOffsetY());        
}

BOOST_AUTO_TEST_CASE(test_gridmap_get)
{
    GridMap grid_map(config);

    Grid<double> &grid = grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());      

    // get existing grid of correct type: OK
    BOOST_CHECK_EQUAL(&(grid_map.getGrid<double>("double_grid")), &grid);

    // get existing grid of wrong type: THROW
    BOOST_CHECK_THROW(grid_map.getGrid<int>("double_grid"), std::exception);

    // get grid with wrong key: THROW
    BOOST_CHECK_THROW(grid_map.getGrid<double>("no_grid"), std::exception); 

    // try to access deleted grid: THROW
    grid_map.removeGrid("double_grid"); 
    BOOST_CHECK_THROW(grid_map.getGrid<double>("double_grid"), std::exception); 
}

BOOST_AUTO_TEST_CASE(test_gridmap_remove)
{
    GridMap grid_map(config);

    grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());       

    // remove grid not existed: false
    BOOST_CHECK_EQUAL(grid_map.removeGrid("no_grid"), false);

    // remove grid existed: true
    BOOST_CHECK_EQUAL(grid_map.removeGrid("double_grid"), true);
}

BOOST_AUTO_TEST_CASE(test_gridmap_removeall)
{
    GridMap grid_map(config);

    grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());       
    grid_map.addGrid<int>("int_grid", std::numeric_limits<int>::infinity());        
    grid_map.addGrid<char>("char_grid", std::numeric_limits<char>::infinity()); 

    grid_map.removeAllGrids();

    // check if the grids exist
    BOOST_CHECK_EQUAL(grid_map.hasGrid("double_grid"), false);  
    BOOST_CHECK_EQUAL(grid_map.hasGrid("int_grid"), false); 
    BOOST_CHECK_EQUAL(grid_map.hasGrid("char_grid"), false);    
}

BOOST_AUTO_TEST_CASE(test_gridmap_keys)
{
    GridMap grid_map(config);

    grid_map.addGrid<double>("double_grid", std::numeric_limits<double>::infinity());       
    grid_map.addGrid<int>("int_grid", std::numeric_limits<int>::infinity());        
    grid_map.addGrid<char>("char_grid", std::numeric_limits<char>::infinity()); 

    std::vector<std::string> keys = grid_map.getAllGridKeys();

    BOOST_CHECK_EQUAL(keys.size(), 3);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "double_grid") !=  keys.end(), true);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "int_grid") !=  keys.end(), true);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "char_grid") !=  keys.end(), true);
}
