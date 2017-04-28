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

#include <maps/LocalMap.hpp>

#include <iostream>

using namespace ::maps;

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
