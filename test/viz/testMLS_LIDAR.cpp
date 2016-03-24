#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/MLSGrid.hpp>

#include "../tools/GeneratePointclouds.hpp"
using namespace ::maps;


BOOST_AUTO_TEST_CASE(mls_simulate_LIDAR)
{
    //    GridConfig conf(150, 150, 0.1, 0.1, -7.5, -7.5);
    Eigen::Vector2d res(0.125, 0.125);
    Vector2ui numCells(200, 200);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //    mls_config.updateModel = MLSConfig::KALMAN;
    mls_config.gapSize = 0.125f;
    mls_config.useNegativeInformation = true;
    MLSGrid *mls = new MLSGrid(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls->getLocalFrame().translation() << 0.5*mls->getSize(), 0;

    /** Equivalent to translate the grid in opposite direction **/
    //Eigen::Vector3d offset_grid;
    //offset_grid << -0.5*mls->getSize(), 0.00;
    //mls->translate(offset_grid);

    typedef Eigen::Hyperplane<double, 3> Plane;
    LIDARSimulator lidar(Eigen::VectorXd::LinSpaced(32, -16*M_PI/180, +16*M_PI/180), Eigen::VectorXd::LinSpaced(360, -M_PI, +M_PI));
    //    LIDARSimulator lidar(Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
    std::vector<Plane> scene;
    {
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();

        for(int i=0; i<3; ++i)
        {
            scene.push_back(Plane(q * Eigen::Vector3d::Unit(i), -2.0));
            scene.push_back(Plane(q * Eigen::Vector3d::Unit(i), 2.0));
        }
    }

    StandaloneVisualizer app;

    Eigen::ArrayXXd ranges;
    PointCloud pointcloud;
    Eigen::Affine3d trafo;
    trafo.setIdentity();
    int loop = 0;
    while (app.wait(1000))
    {
        if(++loop & 1023) continue;
        trafo.translation().setRandom();
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();
        trafo.linear() = q.toRotationMatrix();
        //        std::cout << trafo.matrix() << std::endl;

        lidar.getRanges(ranges, scene, trafo, &pointcloud);

        mls->mergePointCloud(pointcloud, trafo, false);
        app.updateData(*mls);
    }

}
