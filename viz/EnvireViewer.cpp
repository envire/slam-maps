#include <vizkit3d/Vizkit3DWidget.hpp>
#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QLayout>
#include <QHBoxLayout>
#include <iostream>

#include <vizkit3d/GridVisualization.hpp>

#include "MLSGridVisualization.hpp"

#include <math.h>

using namespace envire::maps;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);


    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::KALMAN;
    MLSGrid *mls = new MLSGrid(conf, mls_config);
    for (unsigned int x = 0; x < mls->getCellSizeX(); ++x)
    {
        for (unsigned int y = 0; y < mls->getCellSizeY(); ++y)
        {
            int part = x / 100;

            int rest = x  - 100 *  part;
            float degree = ((float)rest * 360.) / 100.;
            float radian = (degree * M_PI) / 180.;
            double height = std::cos(radian);

            mls->at(x, y).update(SurfacePatch(height, 0.1));
        }
    }

    std::cout << "update finish" << std::endl;


    //create vizkit3d plugin for showing envire
    vizkit3d::MLSGridVisualization *mls_viz = new vizkit3d::MLSGridVisualization();
    mls_viz->updateData(*mls);    

	//create vizkit3d widget
    vizkit3d::Vizkit3DWidget *widget = new vizkit3d::Vizkit3DWidget();
    // grid plugin
	vizkit3d::GridVisualization *grid_viz = new vizkit3d::GridVisualization();    
    widget->addPlugin(grid_viz);
    // add envire plugin
    widget->addPlugin(mls_viz);   

    //create main window
    QMainWindow a;
    //set envire as central dock widget
    a.setCentralWidget(widget); 
    a.show();
    return app.exec();        
}