#ifndef __MAPS_LINESEGMENT_HPP__
#define __MAPS_LINESEGMENT_HPP__

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace maps
{
    template <typename T>
    class LineSegment: public GeometricElement<T>, public Eigen::ParametrizedLine<T,3>
    {
    public:
        LineSegment(Eigen::Matrix<T, 3, 1> &psi_a, Eigen::Matrix<T, 3, 1> &psi_b):
            : Eigen::ParametrizedLine<T,3>(psi_a, (psi_b-psi_a).normalized())
        {
        }


    };
}
#endif /* __MAPS_LINESEGMENT_HPP__ */
