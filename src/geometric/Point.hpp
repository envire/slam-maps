#ifndef __MAPS_POINT_HPP__
#define __MAPS_POINT_HPP__

/** base types **/
#include <base/Float.hpp>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

/** Element type **/
#include <maps/geometric/GeometricElement.hpp>

namespace maps
{
    template <typename T>
    class Point: public GeometricElement< Eigen::Matrix<T, 3, 1, Eigen::DontAlign> >
    {

    public:
        Point(const T &x, const T&y)
            : GeometricElement< Eigen::Matrix<T, 3, 1, Eigen::DontAlign> > (x, y, base::NaN<T>())
        {
        }

        Point(const T &x, const T &y, const T &z)
            : GeometricElement< Eigen::Matrix<T, 3, 1, Eigen::DontAlign> > (x, y, z)
        {
        }

    };
}
#endif /* __MAPS_POINT_HPP__ */
