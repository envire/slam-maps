#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "ElevationMapVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>

using namespace envire::maps;
BOOST_AUTO_TEST_CASE(elevviz_test) 
{
    ElevationMap elev_map(Vector2ui(150, 250), Vector2d(0.1, 0.1));
    elev_map.getLocalFrame().translation() << -0.5 * elev_map.getSize(), 0;

    for (unsigned int x = 10; x < elev_map.getNumCells().x() - 10; ++x) 
    {
        float cs = std::cos(x * M_PI/50);
        for (unsigned int y = 10; y < elev_map.getNumCells().y() - 10; ++y) 
        {
            float sn = std::sin(y * M_PI/50);

            elev_map.at(x,y).elevation = cs * sn;
            elev_map.at(x,y).elevation_min = -1 * cs * sn;
        }
    }

    // set up test environment
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    app.start();

    //create vizkit3d plugin for showing envire
    vizkit3d::ElevationMapVisualization *elev_viz = new vizkit3d::ElevationMapVisualization();
    elev_viz->updateData(elev_map);

    //create vizkit3d widget
    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
    // grid plugin
    vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
    widget->addPlugin(grid_viz);
    // add envire plugin
    widget->addPlugin(elev_viz);

    while (app.isRunning())
    {
        usleep(1000);
    }
}

