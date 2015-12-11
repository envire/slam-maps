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
//    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);
    Eigen::Vector2d res(0.05, 0.05);
    Vector2ui cellSize(300, 300);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSGrid *mls = new MLSGrid(res, cellSize, mls_config);
    mls->offset.translation() << -0.5*mls->getSize(), 0;
    for (unsigned int x = 0; x < cellSize.x(); ++x) for(float dx = 0.; dx <0.99f; dx+=0.125)
    {
        float xx = x+dx;
        float cs = std::cos(xx * M_PI/50);
        for (unsigned int y = 0; y < cellSize.y(); ++y) for (float dy = 0.; dy<0.99; dy+=0.125)
        {
            float yy = y+dy;
            float sn = std::sin(yy* M_PI/50);

            mls->at(x, y).update(SurfacePatch(Eigen::Vector3f(dx*res.x(),dy*res.y(),cs*sn), 0.1));
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
    }
}

BOOST_AUTO_TEST_CASE(mls_loop)
{
//    GridConfig conf(150, 150, 0.1, 0.1, -7.5, -7.5);
    Eigen::Vector2d res(0.1, 0.1);
    Vector2ui numCells(150, 150);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    mls_config.gapSize = 0.05f;
    float R = 5.0f, r=2.05f;
    MLSGrid *mls = new MLSGrid(res, numCells, mls_config);
    mls->offset.translation() << -0.5*mls->getSize(), 0;

    for (float alpha = 0; alpha < M_PI; alpha += M_PI/1024/4)
    {
        float cs = std::cos(alpha);
        float sn = std::sin(alpha);
        for (float beta = 0; beta <= 2*M_PI; beta+=M_PI/256/4)
        {
            // Points of a torus:
            float x = (R+r*std::cos(beta)) * cs;
            float y = (R+r*std::cos(beta)) * sn;
            float z = r*std::sin(beta);

            // Project points into MLS grid:
            Index idx;
            Eigen::Vector2d rem;
            if(!mls->toGrid(Eigen::Vector2d(x,y), idx, rem)) continue;

            mls->at(idx).update(SurfacePatch(Eigen::Vector3f(rem.x(),rem.y(),z), 0.1));
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
    }
}

