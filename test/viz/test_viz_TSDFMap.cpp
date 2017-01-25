#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/TSDFVolumetricMap.hpp>
#include <maps/tools/TSDF_MLSMapReconstruction.hpp>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(tsdf_moving_waves)
{
    Vector3d res(0.2, 0.2, 0.2);
    Vector2ui numCells(40, 40);

    StandaloneVisualizer app;

    TSDFVolumetricMap::Ptr grid(new TSDFVolumetricMap(numCells, res));

    /** Translate the local frame (offset) **/
    grid->getLocalFrame().translation() << 0.5*grid->getSize(), 0;

    maps::tools::TSDF_MLSMapReconstruction mls_reconstruction;
    mls_reconstruction.setTSDFMap(grid);

    Eigen::Vector2d max = 0.375 * grid->getSize();
    Eigen::Vector2d min = -0.375 * grid->getSize();

    double x_shift = 0.;
    while (app.wait(200))
    {
        x_shift += 0.01;
        for (double x = min.x(); x < max.x(); x += 0.05)
        {
            double cs = std::cos(x_shift + x * M_PI/2.5);
            for (double y = min.y(); y < max.y(); y += 0.05)
            {
                double sn = std::sin(y * M_PI/2.5);
                try
                {
                    grid->mergePoint(Eigen::Vector3d(x, y, 4.) + Eigen::Vector3d::Random(), Eigen::Vector3d(x, y, cs*sn));
                }
                catch(std::runtime_error &e)
                {
                    std::cerr << "Caught exception: " << e.what() << std::endl;
                }
            }
        }

        maps::grid::MLSMapPrecalculated mls;
        mls_reconstruction.reconstruct(mls);
        app.updateData(mls);
    }
}