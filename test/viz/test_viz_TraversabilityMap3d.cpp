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
#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/TraversabilityMap3d.hpp>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(traversability_map3d_viz_test)
{
    StandaloneVisualizer app;

    TraversabilityMap3d<TraversabilityNode<double> *> map;

    map.setResolution(Eigen::Vector2d(0.5, 0.5));
    map.resize(Vector2ui(20, 20));

    map.getLocalFrame().translation() << 0.5*map.getSize(), 0;

    TraversabilityNode<double> *child1 = new TraversabilityNode<double>(0.0, Index(1,2));
    map.at(child1->getIndex()).insert(child1);
    TraversabilityNode<double> *child2 = new TraversabilityNode<double>(1.0, Index(1,3));
    map.at(child2->getIndex()).insert(child2);
    TraversabilityNode<double> *child3 = new TraversabilityNode<double>(2.0, Index(1,4));
    map.at(child3->getIndex()).insert(child3);

    TraversabilityNode<double> *parentNode = new TraversabilityNode<double>(12.0, Index(1,10));
    map.at(parentNode->getIndex()).insert(parentNode);

    child1->setType(TraversabilityNodeBase::FRONTIER);
    child1->setType(TraversabilityNodeBase::HOLE);
    child1->setType(TraversabilityNodeBase::OBSTACLE);
    parentNode->setType(TraversabilityNodeBase::TRAVERSABLE);

    parentNode->addConnection(child1);
    child1->addConnection(parentNode);

    parentNode->addConnection(child2);
    child2->addConnection(parentNode);

    parentNode->addConnection(child3);
    child3->addConnection(parentNode);

    TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> base_map = map.copyCast<maps::grid::TraversabilityNodeBase *>();

    while (app.wait(1000))
    {
        app.updateData(base_map);
    }
}
