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

BOOST_AUTO_TEST_CASE(gridviz_test)
{
    GridMap<double> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);
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

BOOST_AUTO_TEST_CASE(gridviz_loop)
{
    GridMap<double> grid_map(Vector2ui(150, 250), Vector2d(0.1, 0.1), -10.);

    /** Translate the local frame (offset) **/
    grid_map.getLocalFrame().translation() << 0.5 * grid_map.getSize(), 0;

    for(unsigned int x = 0; x < grid_map.getNumCells().x(); ++x )
        for(unsigned int y = 0; y < grid_map.getNumCells().y(); ++y)
        {
            float R = Eigen::Vector2d(x-75.,y-125.).cwiseProduct(grid_map.getResolution()).norm();
            float r2 = 2*2 - std::pow(R-5.0, 2);
            if(r2 >= 0)
            {
                grid_map.at(x,y) = std::sqrt(r2);
            }
            else
                grid_map.at(x,y) = 0.0;
        }

    showGridMap(grid_map);
}
