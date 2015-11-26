#define BOOST_TEST_MODULE EnvireVizTest 
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include "MLSGridVisualization.hpp"
#include <vizkit3d/GridVisualization.hpp>

//using namespace envire;
using namespace envire::maps;
BOOST_AUTO_TEST_CASE( mlsviz_test ) 
{
    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSGrid *mls = new MLSGrid(conf, mls_config);
    for (unsigned int x = 0; x < mls->getCellSizeX(); ++x) for(float dx = 0.; dx <0.99f; dx+=0.125)
    {
        float xx = x+dx;
        float cs = std::cos(xx * M_PI/50);
        for (unsigned int y = 0; y < mls->getCellSizeY(); ++y) for (float dy = 0.; dy<0.99; dy+=0.125)
        {
            float yy = y+dy;
            float sn = std::sin(yy* M_PI/50);

            mls->at(x, y).update(SurfacePatch(Eigen::Vector3f(dx*conf.scaleX,dy*conf.scaleY,cs*sn), 0.1));
//            mls->at(x, y).update(SurfacePatch(height, 0.1));
        }
    }

    std::cout << "update finish" << std::endl;

    // set up test environment
     QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
     app.start();

    //create vizkit3d plugin for showing envire
    vizkit3d::MLSGridVisualization *mls_viz = new vizkit3d::MLSGridVisualization();
    mls_viz->updateData(*mls);

        //create vizkit3d widget
    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
    // grid plugin
        vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();
    widget->addPlugin(grid_viz);
    // add envire plugin
    widget->addPlugin(mls_viz);

    while (app.isRunning())
    {
        usleep(1000);
    }}

