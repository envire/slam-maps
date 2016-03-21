#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "ElevationMapVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>

using namespace envire::maps;

static void showElevationMap(const ElevationMap &elev_map)
{
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

            elev_map.at(x,y).elevation = cs * sn;
            elev_map.at(x,y).elevation_min = -1 * cs * sn;
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
            double R = Eigen::Vector2d(x-75.,y-125.).cwiseProduct(elev_map.getResolution()).norm();
            double r2 = 2*2 - std::pow(R-5.0, 2);
            if(r2 >= 0)
            {
                elev_map.at(x,y).elevation = -std::sqrt(r2);
                elev_map.at(x,y).elevation_min = std::sqrt(r2);
            }
            else
                elev_map.at(x,y).elevation = 0.0;
        }

    showElevationMap(elev_map);
}
