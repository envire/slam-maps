
#ifndef TEST_TOOLS_GENERATEPOINTCLOUDS_HPP_
#define TEST_TOOLS_GENERATEPOINTCLOUDS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace maps {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * LIDARSimulator simulates a 3d laser range finder, with laser beams at
 */
class LIDARSimulator
{
public:
    /**
     * Constructor taking vectors of all possible latitude and longitude angles.
     * Both ranges must have at least one value. Latitude must be in [-PI/2, +PI/2], Longitude must be in [-PI, +PI].
     * For a 2D Scanner, choose latitudeRange Eigen::Matrix<double, 1,1>(0.0),
     * for a range of angles use the Eigen::VectorXd::LinSpaced function.
     */
    template<class Derived1, class Derived2>
    LIDARSimulator(const Eigen::MatrixBase<Derived1>& latitudeRange_, const Eigen::MatrixBase<Derived2>& longitudeRange_) : latitudeRange(latitudeRange_), longitudeRange(longitudeRange_)
    {
        assert(latitudeRange.size()>=1    && latitudeRange.abs().maxCoeff() <= M_PI/2.0+0.001 );
        assert(longitudeRange.size() >= 1 && longitudeRange.abs().maxCoeff() <= M_PI+0.001);
    }

    /**
     * getRanges computes ranges to the nearest plane of the scene, given a pose of the LIDAR relative to the scene.
     * Optionally, it also computes the corresponding pointcloud (in the coordinate system of the LIDAR).
     */
    void getRanges(Eigen::Ref<Eigen::ArrayXXd> ranges, const std::vector<Eigen::Hyperplane<double, 3> >& scene, const Eigen::Affine3d & pose, PointCloud *pc = 0)
    {
        Eigen::DenseIndex height = latitudeRange.size();
        Eigen::DenseIndex width = longitudeRange.size();
        assert(ranges.rows()==height && ranges.cols() == width);
        if(pc)
        {
            pc->height = height;
            pc->width = width;
            pc->resize(height * width);
        }
        for(Eigen::DenseIndex i=0; i<latitudeRange.size(); ++i)
        {
            double sinTheta = std::sin(latitudeRange(i)), cosTheta=std::cos(latitudeRange(i));
            for(Eigen::DenseIndex j=0; j<longitudeRange.size(); ++j)
            {
                Eigen::Vector3d dir(cosTheta*std::cos(longitudeRange(j)), cosTheta*std::sin(longitudeRange(j)), sinTheta);
                Eigen::ParametrizedLine<double, 3> line(pose.translation(), pose.linear()*dir);
                double minDist = 1e99;
//                std::cout << "Line = [" << line.origin().transpose() << "] + l*[" << line.direction().transpose() << "]: ";
                for(size_t k=0; k<scene.size(); ++k)
                {
                    double dist = line.intersectionParameter(scene[k]);
//                    std::cout << dist << " ";
                    if(dist > 0.0 && dist < minDist)
                        minDist = dist;
                }
//                std::cout << std::endl;
                ranges(i, j) = minDist;
                if(pc) // PCL first wants the column, then the row:
                    pc->at(j, i).getVector3fMap() = (dir*minDist).cast<float>();
            }
        }
    }

    void getRanges(Eigen::ArrayXXd& ranges, const std::vector<Eigen::Hyperplane<double, 3> >& scene, const Eigen::Affine3d & pose, PointCloud * pc = 0)
    {
        ranges.resize(latitudeRange.size(), longitudeRange.size());
        getRanges(Eigen::Ref<Eigen::ArrayXXd>(ranges), scene, pose, pc);
    }



protected:
    Eigen::ArrayXd latitudeRange, longitudeRange;
};

}  // namespace maps



#endif /* TEST_TOOLS_GENERATEPOINTCLOUDS_HPP_ */
