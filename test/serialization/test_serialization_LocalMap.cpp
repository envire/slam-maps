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
#define BOOST_TEST_MODULE SerializationTest
#include <boost/test/unit_test.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/** Based local map **/
#include <maps/LocalMap.hpp>

using namespace ::maps;

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
    boost::archive::binary_oarchive oa(stream);
    oa << local_map_data_o;

    // deserialize from string stream
    boost::archive::binary_iarchive ia(stream);
    LocalMapData local_map_data_i;
    ia >> local_map_data_i;

    // the deserialized data should be same as in the instance created above
    BOOST_CHECK(local_map_data_o.id == local_map_data_i.id); 
    BOOST_CHECK_EQUAL(local_map_data_o.offset.matrix().isApprox(local_map_data_i.offset.matrix()), true); 
    BOOST_CHECK(local_map_data_o.map_type == local_map_data_i.map_type); 
    BOOST_CHECK(local_map_data_o.EPSG_code == local_map_data_i.EPSG_code); 
}

BOOST_AUTO_TEST_CASE(test_localmap_serialization)
{
    // create an instance of LocalMapData
    boost::shared_ptr<LocalMapData> local_map_data_o(new LocalMapData());
    local_map_data_o->id = "test";
    local_map_data_o->offset = 0.2 * Eigen::Matrix3d::Identity();
    local_map_data_o->map_type = GEOMETRIC_MAP;
    local_map_data_o->EPSG_code = "EPSG_code";

    // create LocalMap with the LocalMapData 
    LocalMap local_map_o(local_map_data_o);

    // serialize
    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << local_map_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    LocalMap local_map_i;
    (*ia) >> local_map_i;

    // the LocalMapData should be the same after the serialization
    boost::shared_ptr<LocalMapData> local_map_data_i = local_map_i.getLocalMapData();

    BOOST_CHECK(local_map_o.getId() == local_map_i.getId()); 
    BOOST_CHECK_EQUAL(local_map_o.getLocalFrame().matrix().isApprox(local_map_i.getLocalFrame().matrix()), true); 
    BOOST_CHECK(local_map_data_o->id == local_map_data_i->id);
    BOOST_CHECK(local_map_data_o->map_type == local_map_data_i->map_type);
    BOOST_CHECK(local_map_data_o->EPSG_code == local_map_data_i->EPSG_code);

    // WARNING: the counter is 2 or more after desirialization,
    // this is bug in boost (1.54)
    BOOST_CHECK(local_map_data_i.use_count() == 3);
    delete ia; 
    // 2: since local variable local_map_data_i and member of local_map_i points to the same
    // LocalMapData instance
    BOOST_CHECK(local_map_data_i.use_count() == 2);
}


