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
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "ElevationMapVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>

using namespace ::maps::grid;

static void showElevationMap(const ElevationMap &elev_map)
{
    // set up test environment
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    app.start();

    //create vizkit3d plugin
    vizkit3d::ElevationMapVisualization *elev_viz = new vizkit3d::ElevationMapVisualization();
    elev_viz->updateData(elev_map);

    //create vizkit3d widget
    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
    // grid plugin
    vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
    widget->addPlugin(grid_viz);
    // add plugin
    widget->addPlugin(elev_viz);

    while (app.isRunning())
    {
        usleep(1000);
    }
}

BOOST_AUTO_TEST_CASE(elevviz_test)
{
    ElevationMap elev_map(Vector2ui(150, 250), Vector2d(0.1, 0.1));
    elev_map.getLocalFrame().translation() << 0.5 * elev_map.getSize(), 0;

    for (unsigned int x = 10; x < elev_map.getNumCells().x() - 10; ++x) 
    {
        float cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < elev_map.getNumCells().y() - 10; ++y) 
        {
            float sn = std::sin(y * M_PI/50);

            elev_map.at(x,y) = cs * sn;
            //elev_map.at(x,y).elevation_min = -1 * cs * sn;
        }
    }
}

BOOST_AUTO_TEST_CASE(elevviz_loop)
{
    ElevationMap elev_map(Vector2ui(150, 250), Vector2d(0.1, 0.1));

    /** Translate the local frame (offset) **/
    elev_map.getLocalFrame().translation() << 0.5 * elev_map.getSize(), 0;

    /** Equivalent to translate the grid in opposite direction **/
//    Eigen::Vector3d offset_grid;
//    offset_grid << -0.5*elev_map.getSize(), 0.00;
//    elev_map.translate(offset_grid);

    for(unsigned int x=0; x< elev_map.getNumCells().x(); ++x )
        for(unsigned int y=0; y<elev_map.getNumCells().y(); ++y)
        {
            float R = Eigen::Vector2d(x-75.,y-125.).cwiseProduct(elev_map.getResolution()).norm();
            float r2 = 2*2 - std::pow(R-5.0, 2);
            if(r2 >= 0)
            {
                elev_map.at(x,y) = std::sqrt(r2);
            }
            else
                elev_map.at(x,y) = 0.0;
        }

    showElevationMap(elev_map);
}
