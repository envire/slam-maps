#include <boost/test/unit_test.hpp>

#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <boost/archive/polymorphic_binary_oarchive.hpp>

#include <envire_maps/LocalMap.hpp>
#include <envire_maps/Grid.hpp>
#include <envire_maps/GridMap.hpp>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_localmap_data_serialization)
{
    // create an instance of LocalMapData
    LocalMapData local_map_data_o;
    local_map_data_o.id = "test";
    local_map_data_o.offset = 0.2 * Eigen::Matrix3d::Identity();
    local_map_data_o.map_type = GEOMETRIC_MAP;
    local_map_data_o.EPSG_code = "EPSG_code";

    // serialize
    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << local_map_data_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive ia(stream);
    LocalMapData local_map_data_i;
    ia >> local_map_data_i;

    // the deserialized data should be same as in the instance created above
    BOOST_CHECK(local_map_data_o.id == local_map_data_i.id); 
    BOOST_CHECK_EQUAL(local_map_data_o.offset.matrix().isApprox(local_map_data_i.offset.matrix()), true); 
    BOOST_CHECK(local_map_data_o.map_type == local_map_data_i.map_type); 
    BOOST_CHECK(local_map_data_o.EPSG_code == local_map_data_i.EPSG_code); 
}

/*BOOST_AUTO_TEST_CASE(test_localmap_serialization)
{
    boost::shared_ptr<LocalMapData> local_map_data_o(new LocalMapData());
    local_map_data_o->id = "test";
    local_map_data_o->offset = 0.2 * Eigen::Matrix3d::Identity();
    local_map_data_o->map_type = GEOMETRIC_MAP;
    local_map_data_o->EPSG_code = "EPSG_code";

    LocalMap local_map_o(local_map_data_o);

    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << local_map_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive *ia = new boost::archive::polymorphic_binary_iarchive(stream);
    LocalMap local_map_i;
    (*ia) >> local_map_i; 

    boost::shared_ptr<LocalMapData> local_map_data_i = local_map_i.getLocalMapData();

    BOOST_CHECK(local_map_o.getId() == local_map_i.getId()); 
    BOOST_CHECK_EQUAL(local_map_o.getLocalFrame().matrix().isApprox(local_map_i.getLocalFrame().matrix()), true); 
    BOOST_CHECK(local_map_data_o->id == local_map_data_i->id);
    BOOST_CHECK(local_map_data_o->map_type == local_map_data_i->map_type);
    BOOST_CHECK(local_map_data_o->EPSG_code == local_map_data_i->EPSG_code);

    // WARNING: the counter is 2 or more after desirialization, 
    // this is bug in boost (1.54)
    BOOST_CHECK(local_map_data_i.use_count() == 2); 
    delete ia; 
    BOOST_CHECK(local_map_data_i.use_count() == 1);  
}

/*BOOST_AUTO_TEST_CASE(test_grid_serialization)
{
    Grid grid_o(Vector2ui(100, 100), Vector2d(0.153, 0.257));

    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << grid_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive ia(stream);
    Grid grid_i;
    ia >> grid_i;     

    BOOST_CHECK_EQUAL(grid_o.getNumCells(), grid_i.getNumCells());
    BOOST_CHECK_EQUAL(grid_o.getResolution(), grid_i.getResolution());    
    BOOST_CHECK_EQUAL(grid_o.getSize().isApprox(grid_i.getSize(), 0.0001), true); 

    boost::shared_ptr<LocalMapData> local_map_data_i = grid_i.getLocalMap();
    boost::shared_ptr<LocalMapData> local_map_data_o = grid_o.getLocalMap();

    BOOST_CHECK(local_map_data_o->id == local_map_data_i->id);
    BOOST_CHECK(local_map_data_o->map_type == local_map_data_i->map_type);
    BOOST_CHECK(local_map_data_o->EPSG_code == local_map_data_i->EPSG_code);   
    BOOST_CHECK_EQUAL(local_map_data_o->offset.matrix().isApprox(local_map_data_i->offset.matrix()), true);  
}

BOOST_AUTO_TEST_CASE(test_gridmap_serialization)
{
    Vector2ui num_cells(100, 200);
    Vector2d resolution(0.1, 0.5);

    GridMap<double> grid_o(num_cells, resolution, default_value); 
    grid_o.getOffset().translate(Eigen::Vector3d::Random(3));

    for (unsigned int x = 0; x < grid_o.getNumCells().x(); ++x)
    {
        for (unsigned int y = 0; y < grid_o.getNumCells().y(); ++y)
        {
            double cell_value = rand();
            grid_o.at(x, y) = cell_value;
        }
    } 

    std::stringstream stream;
    boost::archive::polymorphic_binary_oarchive oa(stream);
    oa << grid_o;

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive ia(stream);
    GridMap<double> grid_i;
    ia >> grid_i;   

    BOOST_CHECK_EQUAL(grid_o.getNumCells(), grid_i.getNumCells());
    BOOST_CHECK_EQUAL(grid_o.getResolution(), grid_i.getResolution());    
    BOOST_CHECK_EQUAL(grid_o.getSize().isApprox(grid_i.getSize(), 0.0001), true);           
}*/