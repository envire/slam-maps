#ifndef __MAPS_POINT_HPP__
#define __MAPS_POINT_HPP__

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility>

namespace maps
{
    /**@brief Point class IEEE 1873 standard
     * adapted to point in D-space.
     * T type (e.g. float or double)
     * D int specification for the dimensional space
     * **/
    template <typename T, int D>
    class Point: public Eigen::Matrix<T, D, 1, Eigen::DontAlign>
    {

    public:
        template <typename... Ts>
        Point(Ts&&... args) : Eigen::Matrix<T, D, 1, Eigen::DontAlign> (std::forward<Ts>(args)...)
        {
        }
    };

    typedef Point<double, 2> Point2d;
    typedef Point<double, 3> Point3d;
    typedef Point<float, 2> Point2f;
    typedef Point<float, 3> Point3f;
}
#endif /* __MAPS_POINT_HPP__ */
