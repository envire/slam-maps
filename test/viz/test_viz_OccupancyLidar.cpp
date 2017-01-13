#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/OccupancyGridMap.hpp>

#include "../tools/GeneratePointclouds.hpp"
using namespace ::maps::grid;
#include <base/TimeMark.hpp>


BOOST_AUTO_TEST_CASE(occupancy_simulate_LIDAR)
{
    Eigen::Vector3d res(0.125, 0.125, 0.05);
    Vector2ui numCells(200, 200);

    OccupancyConfiguration config;
    OccupancyGridMap *grid = new OccupancyGridMap(numCells, res, config);

    /** Translate the local frame (offset) **/
    grid->getLocalFrame().translation() << 0.5*grid->getSize(), 0;

    typedef Eigen::Hyperplane<double, 3> Plane;
    maps::LIDARSimulator lidar(Eigen::VectorXd::LinSpaced(32, -16*M_PI/180, +16*M_PI/180), Eigen::VectorXd::LinSpaced(360, -M_PI, +M_PI));
    //    maps::LIDARSimulator lidar(Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
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
    pointcloud.sensor_origin_ = Eigen::Vector4f::Zero();
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

        base::TimeMark timer("grid->mergePointCloud");
        grid->mergePointCloud(pointcloud, trafo);
        std::cout << timer << std::endl;
        app.updateData(*grid);
    }

    delete grid;
}
