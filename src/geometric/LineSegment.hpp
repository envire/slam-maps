//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef __MAPS_LINESEGMENT_HPP__
#define __MAPS_LINESEGMENT_HPP__

/** Boost serialization **/
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

/** Base types **/
#include <base/Float.hpp>

/** Std **/
#include <math.h>
#include <utility>

namespace maps { namespace geometric
{
    /**@brief LineSegment class IEEE 1873 standard
     * adapted to lines in D-space.
     * T type (e.g. float or double)
     * D int specification for the dimensional space
     * **/
    template <typename T, int D>
    class LineSegment: public Eigen::ParametrizedLine<T, D>
    {
    public:
        typedef typename Eigen::ParametrizedLine<T, D>::VectorType VectorType;

    private:
        /**  Origin (psi_a) and End (psi_b) point of the line 
         * m_origin form the parent class is the psi_a **/
        VectorType _psi_b;

    public:
        inline LineSegment() {}

        LineSegment(const LineSegment<T, D>& other)
            : Eigen::ParametrizedLine<T, D>(other.origin(), other.direction()),
            _psi_b(other.psi_b())
        {}

        LineSegment(const VectorType &psi_a, const VectorType &psi_b)
            : Eigen::ParametrizedLine<T, D>(psi_a, (psi_b-psi_a).normalized()),
            _psi_b(psi_b)
        {}

        inline const VectorType& psi_a() const
        {
            return const_cast<LineSegment<T, D>& >(*this).origin();
        }

        inline VectorType& psi_a()
        {
            return static_cast<VectorType&>(Eigen::ParametrizedLine<T, D>::origin());
        }


        inline const VectorType& psi_b() const
        {
            return const_cast<VectorType& >(_psi_b);
        }

        void psi_b(const VectorType &other_psi_b)
        {
            this->_psi_b = other_psi_b;
            this->direction() =  (this->_psi_b - this->origin()).normalized();
            return;
        }

        inline const VectorType& direction() const
        {
            return const_cast<LineSegment<T, D>& >(*this).direction();
        }

        inline VectorType& direction()
        {
            return static_cast<VectorType&>(Eigen::ParametrizedLine<T, D>::direction());
        }


        /** Perpendicular distance from the origin of the local coordinate
         * system of the map to the line segment. Unit is meter. **/
        inline T rho() const
        {
            /** The distance of zero to its projection onto the line **/
            T dist = this->distance(VectorType::Zero());

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

    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(this->m_origin);
            ar & BOOST_SERIALIZATION_NVP(this->m_direction);
            ar & BOOST_SERIALIZATION_NVP(this->_psi_b);
        }

    };

    typedef LineSegment<double, 2> LineSegment2d;
    typedef LineSegment<double, 3> LineSegment3d;
    typedef LineSegment<float, 2> LineSegment2f;
    typedef LineSegment<float, 3> LineSegment3f;
}}
#endif /* __MAPS_LINESEGMENT_HPP__ */
