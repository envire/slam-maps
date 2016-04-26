#include <boost/test/unit_test.hpp>
#include <maps/grid/LayeredGridMap.hpp>

using namespace ::maps::grid;

Vector2ui num_cells(100, 100);
Vector2d resolution(0.2, 0.5);

// the grid center is 20m x 50m
double offsetX = 10;
double offsetY = 25;

BOOST_AUTO_TEST_CASE(test_mlgridmap_default_constructor)
{
    LayeredGridMap *grid_map = new LayeredGridMap();       

    BOOST_CHECK_EQUAL(grid_map->getLocalFrame().matrix().isApprox(base::Transform3d::Identity().matrix()), true); 

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_mlgridmap_constructor)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    Eigen::Vector3d translation = Eigen::Vector3d::Random(3);
    grid_map->getLocalFrame().translate(translation);   

    BOOST_CHECK_EQUAL(grid_map->getLocalFrame().translation().isApprox(translation), true); 

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_mlgridmap_share_properties)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    grid_map->getId() = "first";

    Eigen::Vector3d translation = Eigen::Vector3d::Random(3);
    grid_map->getLocalFrame().translate(translation);

    // the underlying layers should share the properties with LayeredGridMap
    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());

    GridMap<double> &grid = grid_map->getLayer<double>("double_grid");

    BOOST_CHECK_EQUAL(num_cells, grid.getNumCells());
    BOOST_CHECK_EQUAL(resolution, grid.getResolution());    
    BOOST_CHECK_EQUAL(grid.getSize(), Vector2d(20, 50));
    BOOST_CHECK_EQUAL(grid_map->getId(), grid.getId());      
    BOOST_CHECK_EQUAL(grid_map->getLocalFrame().matrix().isApprox(grid.getLocalFrame().matrix()), true);     

    // change of the properties should apply to LayeredGridMap
    // and its underlying layers
    grid_map->getId() = "grid_map";

    BOOST_CHECK_EQUAL(grid_map->getId(), "grid_map");    
    BOOST_CHECK_EQUAL(grid.getId(), "grid_map"); 

    grid.getId() = "grid";
    BOOST_CHECK_EQUAL(grid_map->getId(), "grid");    
    BOOST_CHECK_EQUAL(grid.getId(), "grid");     

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_gridmap_has)
{   
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);
     
    // check if the not existed grid exists: false
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), false);

    // check if the existed grid exists: true
    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), true);

    GridMap<double> &grid = grid_map->getLayer<double>("double_grid");

    // check if the existence of the grid after it was removed
    grid_map->removeLayer("double_grid"); 
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), false);
    BOOST_CHECK_EQUAL(grid.getId(),  maps::UNKNOWN_MAP_ID);

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_add)
{   
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    // add grid that does not exist until now: OK
    BOOST_CHECK_NO_THROW(grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity())); 

    // add grid with the key existed already and of the same type: THROW
    BOOST_CHECK_THROW(grid_map->addLayer<double>("double_grid",  std::numeric_limits<double>::infinity()), std::exception);

    // add grid with the key existed already but of the different type: THROW
    BOOST_CHECK_THROW(grid_map->addLayer<int>("double_grid",  std::numeric_limits<int>::infinity()), std::exception);

    // add grid with the key not existed and of the different type: true
    BOOST_CHECK_NO_THROW(grid_map->addLayer<int>("int_grid",  std::numeric_limits<int>::infinity()));

    delete grid_map;     
}



BOOST_AUTO_TEST_CASE(test_gridmap_get)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    GridMap<double> &grid = grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());      

    // get existing grid of correct type: OK
    BOOST_CHECK_EQUAL(&(grid_map->getLayer<double>("double_grid")), &grid);

    // get existing grid of wrong type: THROW
    BOOST_CHECK_THROW(grid_map->getLayer<int>("double_grid"), std::exception);

    // get grid with wrong key: THROW
    BOOST_CHECK_THROW(grid_map->getLayer<double>("no_grid"), std::exception); 

    // try to access deleted grid: THROW
    grid_map->removeLayer("double_grid"); 
    BOOST_CHECK_THROW(grid_map->getLayer<double>("double_grid"), std::exception); 

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_remove)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());       

    // remove grid not existed: false
    BOOST_CHECK_EQUAL(grid_map->removeLayer("no_grid"), false);

    // remove grid existed: true
    BOOST_CHECK_EQUAL(grid_map->removeLayer("double_grid"), true);

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_removeall)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());       
    grid_map->addLayer<int>("int_grid", std::numeric_limits<int>::infinity());        
    grid_map->addLayer<char>("char_grid", std::numeric_limits<char>::infinity()); 

    grid_map->removeAllLayers();

    // check if the grids exist
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), false);  
    BOOST_CHECK_EQUAL(grid_map->hasLayer("int_grid"), false); 
    BOOST_CHECK_EQUAL(grid_map->hasLayer("char_grid"), false);    

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_keys)
{
    LayeredGridMap *grid_map = new LayeredGridMap(num_cells, resolution);

    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());       
    grid_map->addLayer<int>("int_grid", std::numeric_limits<int>::infinity());        
    grid_map->addLayer<char>("char_grid", std::numeric_limits<char>::infinity()); 

    std::vector<std::string> keys = grid_map->getAllLayerKeys();

    BOOST_CHECK_EQUAL(keys.size(), 3);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "double_grid") !=  keys.end(), true);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "int_grid") !=  keys.end(), true);
    BOOST_CHECK_EQUAL(std::find(keys.begin(), keys.end(), "char_grid") !=  keys.end(), true);

    delete grid_map;
}
