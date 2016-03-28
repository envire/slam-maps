#ifndef __MAPS_LINESEGMENT_HPP__
#define __MAPS_LINESEGMENT_HPP__

/** Base types **/
#include <base/Float.hpp>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <math.h>
#include <utility>

namespace maps
{
    /**@brief LineSegment class IEEE 1873 standard
     * adapted to lines in D-space.
     * T type (e.g. float or double)
     * D int specification for the dimensional space
     * **/
    template <typename T, int D>
    class LineSegment: public Eigen::ParametrizedLine<T, D>
    {
    private:
        /**  Origin (psi_a) and End (psi_b) point of the line **/
        Eigen::Matrix<T, D, 1, Eigen::DontAlign> _psi_b;

    public:
        LineSegment(Eigen::Matrix<T, D, 1, Eigen::DontAlign> &psi_a, Eigen::Matrix<T, D, 1, Eigen::DontAlign> &psi_b)
            : Eigen::ParametrizedLine<T, D>(psi_a, (psi_b-psi_a).normalized()),
            _psi_b(psi_b)
        {
        }

        inline Eigen::Matrix<T, D, 1, Eigen::DontAlign> psi_a() const
        {
            return this->origin();
        }

        inline Eigen::Matrix<T, D, 1, Eigen::DontAlign> psi_b() const
        {
            return _psi_b;
        }


        /** Perpendicular distance from the origin of the local coordinate
         * system of the map to the line segment. Unit is meter. **/
        inline T rho() const
        {
            /** The distance of zero to its projection onto the line **/
            T dist = this->distance(Eigen::Matrix< T, D, 1,  Eigen::DontAlign>::Zero());

            /** In case the projected distance is zero the line is passing by
             * the origin of the local coordinate system**/
            if (dist == 0.00)
            {
                dist += this->origin().norm();
            }

            return dist;
        }

        /** Angle in radians [0, 2*pi] for the orientation of a vector
         * representing the perpendicular distance to the line segment, measured
         * counterclockwise from horizontal x-axis of the local coordinate
         * system of the map.**/
        T alpha() const
        {

            /** The slope is given when the dimension D equal to 2**/
            if (D == 2)
            {
                /** Normally Alpha is the angle given by the slope m = tan(alpha) **/
                T alpha = atan2(this->direction()[1], this->direction()[0]);
                //std::cout<<"direction[0]: "<<this->direction()[0] <<"\tdirection[1]: "<<this->direction()[1]<<"\n";

                /** Convert alpha between [0, 2pi] **/
                alpha = (alpha >= 0 ? alpha : (2.0*M_PI + alpha));

                /** IEEE Std 1873-2015 request alpha as the perpendicular to it
                 * It means the angle of the rho_axis wrt to the x-axis of the
                 * local coordinate frame **/
                alpha += M_PI/2.0;

                /** Convert between [0, 2*pi] **/
                return (alpha > 2.0*M_PI ? alpha - (2.0*M_PI): alpha);
            }
            return base::NaN<T>();
        }

    };

    typedef LineSegment<double, 2> LineSegment2d;
    typedef LineSegment<double, 3> LineSegment3d;
    typedef LineSegment<float, 2> LineSegment2f;
    typedef LineSegment<float, 3> LineSegment3f;
}
#endif /* __MAPS_LINESEGMENT_HPP__ */
