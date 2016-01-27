
#include <boost/test/unit_test.hpp>

#include "tools/GeneratePointclouds.hpp"


using namespace envire::maps;

BOOST_AUTO_TEST_CASE(test_generate_PC)
{
    typedef Eigen::Hyperplane<double, 3> Plane;
    LIDARSimulator lidar(Eigen::VectorXd::LinSpaced(20, -1.0, +1.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
//    LIDARSimulator lidar(Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::LinSpaced(30, -M_PI, +M_PI));
    std::vector<Plane> scene;
    for(int i=0; i<3; ++i)
    {
        scene.push_back(Plane(Eigen::Vector3d::Unit(i), 2.0));
        scene.push_back(Plane(Eigen::Vector3d::Unit(i), -2.0));
    }
    Eigen::ArrayXXd ranges;
    PointCloud pointcloud;
    Eigen::Affine3d trafo;
    trafo.setIdentity();
    for(int k=0; k<5; ++k)
    {
        trafo.translation().setRandom();
        Eigen::Quaterniond q; q.coeffs().setRandom(); q.normalize();
        trafo.linear() = q.toRotationMatrix();
//        std::cout << trafo.matrix() << std::endl;

        lidar.getRanges(ranges, scene, trafo, &pointcloud);

        ranges.transposeInPlace();

        for(PointCloud::const_iterator it = pointcloud.begin(); it != pointcloud.end(); ++it)
        {
//            std::cout << ranges(it - pointcloud.begin()) << ",\t" << it->getVector3fMap().transpose() << std::endl;
            BOOST_CHECK_CLOSE((trafo * it->getVector3fMap().cast<double>()).lpNorm<Eigen::Infinity>(), 2.0, 0.01);
        }

    }
}


