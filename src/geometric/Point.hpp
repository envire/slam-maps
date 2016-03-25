#ifndef __MAPS_POINT_HPP__
#define __MAPS_POINT_HPP__

/** base types **/
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace maps
{
    template <typename T>
    class Point: public GeometricElement<T>, public Eigen::Matrix<T, 3, 1, Eigen::DontAlign>
    {

    public:
        Point(T &x, T &y, T &z)
            : base::Point(x, y, z)
        {
        }

    };
}
#endif /* __MAPS_POINT_HPP__ */
