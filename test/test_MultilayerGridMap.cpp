#include <boost/test/unit_test.hpp>
#include <envire_maps/MultilayerGridMap.hpp>

using namespace envire::maps;

Vector2ui num_cells(100, 100);
Vector2d resolution(0.2, 0.5);

// the grid center is 20m x 50m
double offsetX = 10;
double offsetY = 25;

BOOST_AUTO_TEST_CASE(test_mlgridmap_default_constructor)
{
    MultilayerGridMap *grid_map = new MultilayerGridMap();   

    BOOST_CHECK_EQUAL(grid_map->getNumCells(), Vector2ui(0, 0));
    BOOST_CHECK_EQUAL(grid_map->getResolution(), Vector2d(0, 0));    
    BOOST_CHECK_EQUAL(grid_map->getSize(), Vector2d(0, 0));     

    BOOST_CHECK_EQUAL(grid_map->localFrame().matrix().isApprox(base::Transform3d::Identity().matrix()), true); 

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_mlgridmap_constructor)
{
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

    Eigen::Vector3d translation = Eigen::Vector3d::Random(3);
    grid_map->localFrame().translate(translation);

    BOOST_CHECK_EQUAL(grid_map->getNumCells(), num_cells);
    BOOST_CHECK_EQUAL(grid_map->getResolution(), resolution);    
    BOOST_CHECK_EQUAL(grid_map->getSize(), Vector2d(20, 50));     

    BOOST_CHECK_EQUAL(grid_map->localFrame().translation().isApprox(translation), true); 

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_mlgridmap_share_properties)
{
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

    grid_map->id() = "first";

    Eigen::Vector3d translation = Eigen::Vector3d::Random(3);
    grid_map->localFrame().translate(translation);

    // the underlying layers should share the properties with MultilayerGridMap
    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());

    GridMap<double> &grid = grid_map->getLayer<double>("double_grid");

    BOOST_CHECK_EQUAL(grid_map->getNumCells(), grid.getNumCells());
    BOOST_CHECK_EQUAL(grid_map->getResolution(), grid.getResolution());       
    BOOST_CHECK_EQUAL(grid_map->id(), grid.id());      
    BOOST_CHECK_EQUAL(grid_map->localFrame().matrix().isApprox(grid.localFrame().matrix()), true);     

    // change of the properties should apply to MultilayerGridMap
    // and its underlying layers
    grid_map->id() = "grid_map";

    BOOST_CHECK_EQUAL(grid_map->id(), "grid_map");    
    BOOST_CHECK_EQUAL(grid.id(), "grid_map"); 

    grid.id() = "grid";
    BOOST_CHECK_EQUAL(grid_map->id(), "grid");    
    BOOST_CHECK_EQUAL(grid.id(), "grid");     

    delete grid_map;        
}

BOOST_AUTO_TEST_CASE(test_gridmap_has)
{   
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);
     
    // check if the not existed grid exists: false
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), false);

    // check if the existed grid exists: true
    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), true);

    GridMap<double> &grid = grid_map->getLayer<double>("double_grid");

    // check if the existence of the grid after it was removed
    grid_map->removeLayer("double_grid"); 
    BOOST_CHECK_EQUAL(grid_map->hasLayer("double_grid"), false);
    BOOST_CHECK_EQUAL(grid.id(), UNKNOWN_MAP_ID);

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_add)
{   
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

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
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

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
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

    grid_map->addLayer<double>("double_grid", std::numeric_limits<double>::infinity());       

    // remove grid not existed: false
    BOOST_CHECK_EQUAL(grid_map->removeLayer("no_grid"), false);

    // remove grid existed: true
    BOOST_CHECK_EQUAL(grid_map->removeLayer("double_grid"), true);

    delete grid_map;
}

BOOST_AUTO_TEST_CASE(test_gridmap_removeall)
{
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

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
    MultilayerGridMap *grid_map = new MultilayerGridMap(num_cells, resolution);

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
