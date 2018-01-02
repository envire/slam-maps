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

#include <maps/grid/TraversabilityMap3d.hpp>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(test_copyCast)
{
    boost::shared_ptr<maps::LocalMapData> data(new maps::LocalMapData());
    TraversabilityMap3d<TraversabilityNode<double> *> map(Vector2ui(2,2), Eigen::Vector2d(1,1), data);

    Index idx(0,0);
    map.at(idx).insert(new TraversabilityNode<double>(5.0, idx));
    
    //fun fact, here the move assign ist optimized out.
    //smart compiler...
    TraversabilityBaseMap3d baseMap = map.copyCast<TraversabilityNodeBase *>();
    
    BOOST_CHECK_EQUAL(map.at(idx).size(), 1);
    BOOST_CHECK_EQUAL(baseMap.at(idx).size(), 1);
    
}

BOOST_AUTO_TEST_CASE(test_copyCastNonStack)
{
    boost::shared_ptr<maps::LocalMapData> data(new maps::LocalMapData());
    TraversabilityMap3d<TraversabilityNode<double> *> *map = new TraversabilityMap3d<TraversabilityNode<double> *>(Vector2ui(2,2), Eigen::Vector2d(1,1), data);

    Index idx(0,0);
    map->at(idx).insert(new TraversabilityNode<double>(5.0, idx));
    
    TraversabilityBaseMap3d * baseMap = new TraversabilityBaseMap3d();
    //test move assignment
    *baseMap = map->copyCast<TraversabilityNodeBase *>();
    
    BOOST_CHECK_EQUAL(map->at(idx).size(), 1);
    BOOST_CHECK_EQUAL(baseMap->at(idx).size(), 1);
    
    //move constructor
    TraversabilityBaseMap3d *baseMapCA(new TraversabilityBaseMap3d(map->copyCast<TraversabilityNodeBase *>()));

    BOOST_CHECK_EQUAL(map->at(idx).size(), 1);
    BOOST_CHECK_EQUAL(baseMapCA->at(idx).size(), 1);
    
    delete map;
    delete baseMap;
    delete baseMapCA;
}

BOOST_AUTO_TEST_CASE(test_deepCopy)
{
    boost::shared_ptr<maps::LocalMapData> data(new maps::LocalMapData());
    TraversabilityMap3d<TraversabilityNode<double> *> map(Vector2ui(2,2), Eigen::Vector2d(1,1), data);

    Index idx(0,0);
    
    map.at(idx).insert(new TraversabilityNode<double>(5.0, idx));

    //copy constructor case
    TraversabilityMap3d<TraversabilityNode<double> *> copy(map);

    BOOST_CHECK_EQUAL(map.at(idx).size(), 1);
    BOOST_CHECK_EQUAL(copy.at(idx).size(), 1);
    
    BOOST_CHECK(*map.at(idx).begin() != *copy.at(idx).begin());
    
    copy.clear();
    BOOST_CHECK_EQUAL(copy.at(idx).size(), 0);
    BOOST_CHECK_EQUAL(map.at(idx).size(), 1);

    //assignment operator
    copy = map;
    BOOST_CHECK_EQUAL(copy.at(idx).size(), 1);
    
    BOOST_CHECK(*map.at(idx).begin() != *copy.at(idx).begin());
}

