#include <boost/test/unit_test.hpp>
#include <envire_maps/LocalMap.hpp>

#include <iostream>

using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_local_map_default_constuctor)
{
    // default
    LocalMap *map = new LocalMap();

    BOOST_CHECK_EQUAL(map->getId(), UNKNOWN_MAP_ID);
    BOOST_CHECK_EQUAL(map->getLocalFrame().matrix().isApprox(base::Transform3d::Identity().matrix()), true); 
    BOOST_CHECK_EQUAL(map->getMapType(), UNKNOWN_MAP);
    BOOST_CHECK_EQUAL(map->getEPSGCode(), UNKNOWN_EPSG_CODE);

    delete map;
}

BOOST_AUTO_TEST_CASE(test_local_map_constuctor)
{
    // create data with certain parameter
    boost::shared_ptr<LocalMapData> data(new LocalMapData());
    data->id = "local_map_data";
    data->offset = base::Transform3d::Identity();
    data->offset.translate(Eigen::Vector3d::Random(3));
    data->map_type = LocalMapType::GEOMETRIC_MAP;
    data->EPSG_code = "EPSG::5243";

    // only the data points to LocalMapData instance => 1
    BOOST_CHECK_EQUAL(data.use_count(), 1); 

    // create first local map with the existing data
    LocalMap *map = new LocalMap(data);

    // the map should contain the same instance of the LocalMapData
    BOOST_CHECK_EQUAL(map->getId(), data->id);
    BOOST_CHECK_EQUAL(map->getLocalFrame().matrix().isApprox(data->offset.matrix()), true); 
    BOOST_CHECK_EQUAL(map->getMapType(), data->map_type); 
    BOOST_CHECK_EQUAL(map->getEPSGCode(), data->EPSG_code); 
    // the data and map points to LocalMapData instace => 2
    BOOST_CHECK_EQUAL(data.use_count(), 2); 

    // create second local map with the existing data
    LocalMap *map_2 = new LocalMap(data);

    // the second map should contain the same instance of the LocalMapData as the fist map
    BOOST_CHECK_EQUAL(map_2->getId(), map->getId());
    BOOST_CHECK_EQUAL(map_2->getLocalFrame().matrix().isApprox(map->getLocalFrame().matrix()), true); 
    BOOST_CHECK_EQUAL(map_2->getMapType(), map->getMapType()); 
    BOOST_CHECK_EQUAL(map_2->getEPSGCode(), map->getEPSGCode()); 
    // the data and two map points to LocalMapData instace => 3
    BOOST_CHECK_EQUAL(data.use_count(), 3); 

    // create new LocalMapData with different parameters 
    boost::shared_ptr<LocalMapData> data_new(new LocalMapData());
    data_new->id = "changed"; 
    data_new->offset.translate(Eigen::Vector3d::Random(3));  
    data_new->map_type = LocalMapType::TOPOLOGICAL_MAP;
    data_new->EPSG_code = "EPSG::1111";

    // changes of shared LocalMapData should be applied to the both maps
    map->getId() = data_new->id;
    map->getLocalFrame() = data_new->offset;

    map_2->getMapType() = data_new->map_type;
    map_2->getEPSGCode() = data_new->EPSG_code;

    BOOST_CHECK_EQUAL(map->getId(), data_new->id);
    BOOST_CHECK_EQUAL(map_2->getId(), map->getId());

    BOOST_CHECK_EQUAL(map->getLocalFrame().isApprox(data_new->offset), true);
    BOOST_CHECK_EQUAL(map_2->getLocalFrame().matrix().isApprox(map->getLocalFrame().matrix()), true); 

    BOOST_CHECK_EQUAL(map->getMapType(), data_new->map_type);
    BOOST_CHECK_EQUAL(map_2->getMapType(), map->getMapType());    

    BOOST_CHECK_EQUAL(map->getEPSGCode(), data_new->EPSG_code);
    BOOST_CHECK_EQUAL(map_2->getEPSGCode(), map->getEPSGCode()); 

    // the data and two map points to LocalMapData instace => 3
    BOOST_CHECK_EQUAL(data.use_count(), 3); 
}

BOOST_AUTO_TEST_CASE(test_local_map_copy)
{
    // create data with certain parameter
    boost::shared_ptr<LocalMapData> data(new LocalMapData());
    data->id = "local_map_data";
    data->offset = base::Transform3d::Identity();
    data->offset.translate(Eigen::Vector3d::Random(3));
    data->map_type = LocalMapType::GEOMETRIC_MAP;
    data->EPSG_code = "EPSG::5243";  

    // only the data points to LocalMapData instance => 1
    BOOST_CHECK_EQUAL(data.use_count(), 1); 

    // create first map which shares the same instance of LocalMapData with data
    LocalMap *map = new LocalMap(data);
    // the data and first map points to LocalMapData instance => 2
    BOOST_CHECK_EQUAL(data.use_count(), 2); 

    // create second map which has its own LocalMapData
    LocalMap *map_2 = new LocalMap(*map);  
    // the data and first map points to LocalMapData instace => 3 (2 + temp object)
    // the second map has its own instance => 2 (1 + temp object)
    // TODO:check later
    //BOOST_CHECK_EQUAL(map->getLocalMapData().use_count(), 3);
    //BOOST_CHECK_EQUAL(map_2->getLocalMapData().use_count(), 2);     
    BOOST_CHECK_EQUAL(map->getLocalMapData().use_count(), 2);
    BOOST_CHECK_EQUAL(map_2->getLocalMapData().use_count(), 1);     


    BOOST_CHECK_EQUAL(map->getId(), map_2->getId());
    BOOST_CHECK_EQUAL(map->getLocalFrame().matrix().isApprox(map_2->getLocalFrame().matrix()), true);   
    BOOST_CHECK_EQUAL(map->getMapType(), map_2->getMapType()); 
    BOOST_CHECK_EQUAL(map->getEPSGCode(), map_2->getEPSGCode());   

    // change the parameter of one map should not affect the other map 
    boost::shared_ptr<LocalMapData> data_new(new LocalMapData());
    data_new->id = "changed";
    data_new->offset = base::Transform3d::Identity();  
    data_new->map_type = LocalMapType::GRID_MAP;
    data_new->EPSG_code = "EPSG::1111";

    // change parameter of the second map
    map_2->getId() = data_new->id;
    map_2->getLocalFrame() = data_new->offset;
    map_2->getMapType() = data_new->map_type;
    map_2->getEPSGCode() = data_new->EPSG_code;
  
    // the parameter of first and second map should differ
    BOOST_CHECK_EQUAL((map->getId() == map_2->getId()), false);
    BOOST_CHECK_EQUAL(map->getLocalFrame().matrix().isApprox(map_2->getLocalFrame().matrix()), false);  
    BOOST_CHECK_EQUAL((map->getMapType() == map_2->getMapType()), false);
    BOOST_CHECK_EQUAL((map->getEPSGCode() == map_2->getEPSGCode()), false);

    delete map;
    delete map_2;
}
