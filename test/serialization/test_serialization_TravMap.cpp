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

#include <maps/grid/TraversabilityMap3d.hpp>

#include <fstream>

using namespace ::maps::grid;

void checkEqual(TraversabilityNodeBase &node, TraversabilityNodeBase &node_out)
{
    BOOST_CHECK(node.getHeight() == node_out.getHeight()); 
    BOOST_CHECK(node.getIndex() == node_out.getIndex());
    BOOST_CHECK(node.getType() == node_out.getType());
    BOOST_CHECK(node.isExpanded() == node_out.isExpanded());
}

BOOST_AUTO_TEST_CASE(test_travnodebase_serialization)
{
    
    TraversabilityNodeBase node(10.0, Index(1,2));
    
    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << node;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    TraversabilityNodeBase node_out(0, Index(0,0));
    (*ia) >> node_out;

    checkEqual(node, node_out);
}

void checkEqual(TraversabilityNode<double> &node, TraversabilityNode<double> &node_out)
{
    checkEqual(dynamic_cast<TraversabilityNodeBase &>(node), dynamic_cast<TraversabilityNodeBase &>(node_out));
    BOOST_CHECK(node.getUserData() == node_out.getUserData());
}

BOOST_AUTO_TEST_CASE(test_TraversabilityNode_serialization)
{
    
    TraversabilityNode<double> node(10.0, Index(1,2));
    
    node.getUserData() = 45;
    
    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << node;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    TraversabilityNode<double> node_out(0, Index(0,0));
    (*ia) >> node_out;

    checkEqual(node, node_out);
}

BOOST_AUTO_TEST_CASE(test_TraversabilityMap_serialization)
{
    TraversabilityMap3d<TraversabilityNode<double> *> map;
    
    map.resize(Vector2ui(20, 20));
    
    TraversabilityNode<double> *child1 = new TraversabilityNode<double>(10.0, Index(1,2));
    map.at(child1->getIndex()).insert(child1);
    TraversabilityNode<double> *child2 = new TraversabilityNode<double>(11.0, Index(1,3));
    map.at(child2->getIndex()).insert(child2);
    TraversabilityNode<double> *child3 = new TraversabilityNode<double>(12.0, Index(1,4));
    map.at(child3->getIndex()).insert(child3);

    TraversabilityNode<double> *parentNode = new TraversabilityNode<double>(22.0, Index(1,10));
    map.at(parentNode->getIndex()).insert(parentNode);

    parentNode->getUserData() = 45;

    child1->getUserData() = 85;
    child2->getUserData() = 55;
    child3->getUserData() = 65;

    parentNode->addConnection(child1);
    child1->addConnection(parentNode);

    parentNode->addConnection(child2);
    child2->addConnection(parentNode);

    parentNode->addConnection(child3);
    child3->addConnection(parentNode);



    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << map;

    // deserialize from string stream
    boost::archive::binary_iarchive ia(stream);
    TraversabilityMap3d<TraversabilityNode<double> *> mapOut;
    ia >> mapOut;

    BOOST_CHECK_EQUAL(mapOut.at(parentNode->getIndex()).size(), 1);

    TraversabilityNode<double> *node_out = *mapOut.at(parentNode->getIndex()).begin();

    checkEqual(*parentNode, *node_out);

    BOOST_CHECK(parentNode->getConnections().size() == node_out->getConnections().size());

    TraversabilityNodeBase *childOut1 = node_out->getConnections()[0];
    BOOST_CHECK(childOut1->getConnections().size() == 1);
    TraversabilityNodeBase *parentOut = childOut1->getConnections()[0];

    //test that the out node points towards an new object
    BOOST_CHECK(parentNode != parentOut);

    BOOST_CHECK(node_out == parentOut);
}

BOOST_AUTO_TEST_CASE(test_TraversabilityMap_serialization_file)
{
    // Create Map.
    TraversabilityMap3d<TraversabilityNode<double> *> map;

    map.resize(Vector2ui(20, 20));

    TraversabilityNode<double> *child1 = new TraversabilityNode<double>(10.0, Index(1,2));
    map.at(child1->getIndex()).insert(child1);
    TraversabilityNode<double> *child2 = new TraversabilityNode<double>(11.0, Index(1,3));
    map.at(child2->getIndex()).insert(child2);
    TraversabilityNode<double> *child3 = new TraversabilityNode<double>(12.0, Index(1,4));
    map.at(child3->getIndex()).insert(child3);

    TraversabilityNode<double> *parentNode = new TraversabilityNode<double>(22.0, Index(1,10));
    map.at(parentNode->getIndex()).insert(parentNode);

    parentNode->getUserData() = 45;

    child1->getUserData() = 85;
    child2->getUserData() = 55;
    child3->getUserData() = 65;

    parentNode->addConnection(child1);
    child1->addConnection(parentNode);

    parentNode->addConnection(child2);
    child2->addConnection(parentNode);

    parentNode->addConnection(child3);
    child3->addConnection(parentNode);

    // Serialization.
    std::string filename = "TravMap_v0.bin";

    // Save.
    std::ofstream output(filename, std::ios::binary);
    if (output.is_open())
    {
        boost::archive::binary_oarchive oa(output);

        oa << map;
    }

    // Load.
    std::ifstream input(filename, std::ios::binary);
    if (input.is_open())
    {
        boost::archive::binary_iarchive ia(input);

        ia >> map;
    }

    // Check for errors.
    BOOST_CHECK_EQUAL(map.at(parentNode->getIndex()).size(), 1);

    TraversabilityNode<double> *node_out = *map.at(parentNode->getIndex()).begin();

    checkEqual(*parentNode, *node_out);

    BOOST_CHECK(parentNode->getConnections().size() == node_out->getConnections().size());

    TraversabilityNodeBase *childOut1 = node_out->getConnections()[0];
    BOOST_CHECK(childOut1->getConnections().size() == 1);
    TraversabilityNodeBase *parentOut = childOut1->getConnections()[0];

    //test that the out node points towards an new object
    BOOST_CHECK(parentNode != parentOut);

    BOOST_CHECK(node_out == parentOut);
}

// Tests backwards compatibility to version without forced pointers.
BOOST_AUTO_TEST_CASE(test_TraversabilityMap_serialization_backwards_compatibility)
{
    std::string filename = "TravMap_v00.bin";

    TraversabilityMap3d<TraversabilityNode<double>*> map;

    // Load.
    std::ifstream input(filename, std::ios::binary);
    if (input.is_open())
    {
        boost::archive::binary_iarchive ia(input);

        ia >> map;
    }

    // Check for errors.
    BOOST_CHECK_EQUAL(map.getNumCells().x(), 20);
    BOOST_CHECK_EQUAL(map.getNumCells().y(), 20);

    BOOST_CHECK_EQUAL(map.at(Index(1, 10)).size(), 1);

    TraversabilityNode<double> *node_out = *map.at(1, 10).begin();

    BOOST_CHECK_EQUAL(node_out->getConnections().size(), 3);

    TraversabilityNodeBase *childOut1 = node_out->getConnections()[0];
    BOOST_CHECK(childOut1->getConnections().size() == 1);
    TraversabilityNodeBase *parentOut = childOut1->getConnections()[0];
    BOOST_CHECK(node_out == parentOut);
}
