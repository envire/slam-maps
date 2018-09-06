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
#include <vizkit3d/GridMapVisualization.hpp>

using namespace ::maps::grid;

template <typename CellT>
static void showGridMap(const GridMap<CellT> &grid_map)
{
    // set up test environment
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    app.start();

    //create vizkit3d plugin
    vizkit3d::GridMapVisualization *grid_viz = new vizkit3d::GridMapVisualization();
    grid_viz->updateData(grid_map);

    //create vizkit3d widget
    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
    // add plugin
    widget->addPlugin(grid_viz);

    while (app.isRunning())
    {
        usleep(1000);
    }
}

BOOST_AUTO_TEST_CASE(gridviz_test_min)
{
    Vector2ui numCells(4, 4);
    Vector2d resolution(0.05, 0.05);

    GridMap<double> grid_map(numCells, resolution, -0.01);

    for (unsigned int y = 0; y < grid_map.getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid_map.getNumCells().x(); ++x)
        {
            if (x == 0 || y == 0 || x == numCells.x() - 1 || y  == numCells.y() - 1)
                grid_map.at(x, y) = -0.01;
            else
                grid_map.at(x, y) = 0.001 * (x + y) - 0.01;
        }
    }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_test_min_full)
{
    Vector2ui numCells(8, 10);
    Vector2d resolution(0.01, 0.02);

    GridMap<double> grid_map(numCells, resolution, -0.01);

    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for (unsigned int y = 0; y < grid_map.getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < grid_map.getNumCells().x(); ++x)
        {
                grid_map.at(x, y) = 0.001 * (x + y) - 0.01;
        }
    }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_test)
{
    GridMap<double> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for (unsigned int x = 10; x < grid_map.getNumCells().x() - 10; ++x) 
    {
        double cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < grid_map.getNumCells().y() - 10; ++y) 
        {
            double sn = std::sin(y * M_PI/50);

            grid_map.at(x,y) = cs * sn;
        }
    }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_loop)
{
    GridMap<double> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);

    /** Translate the local frame (offset) **/
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for(unsigned int x = 0; x < grid_map.getNumCells().x(); ++x )
        for(unsigned int y = 0; y < grid_map.getNumCells().y(); ++y)
        {
            double R = Eigen::Vector2d(x-75.,y-125.).cwiseProduct(grid_map.getResolution()).norm();
            double r2 = 2*2 - std::pow(R-5.0, 2);
            if(r2 >= 0)
            {
                grid_map.at(x,y) = std::sqrt(r2);
            }
            else
                grid_map.at(x,y) = 0.0;
        }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_float_test)
{
    GridMap<float> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for (unsigned int x = 10; x < grid_map.getNumCells().x() - 10; ++x) 
    {
        float cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < grid_map.getNumCells().y() - 10; ++y) 
        {
            float sn = std::sin(y * M_PI/50);

            grid_map.at(x,y) = cs * sn;
        }
    }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_int_test)
{
    GridMap<int> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for (unsigned int x = 10; x < grid_map.getNumCells().x() - 10; ++x) 
    {
        float cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < grid_map.getNumCells().y() - 10; ++y) 
        {
            float sn = std::sin(y * M_PI/50);

            grid_map.at(x,y) = cs * sn * 10;
        }
    }

    showGridMap(grid_map);
}

BOOST_AUTO_TEST_CASE(gridviz_char_test)
{
    GridMap<char> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for (unsigned int x = 10; x < grid_map.getNumCells().x() - 10; ++x) 
    {
        float cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < grid_map.getNumCells().y() - 10; ++y) 
        {
            float sn = std::sin(y * M_PI/50);

            grid_map.at(x,y) = cs * sn * 10;
        }
    }

    showGridMap(grid_map);
}




