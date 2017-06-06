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
#ifndef __MAPS_INDEX_HPP__
#define __MAPS_INDEX_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/serialization/access.hpp>

namespace maps { namespace grid
{
    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;

    typedef Eigen::Vector2i Vector2i; 

    /**@brief type for grid resolution
     */
    typedef Eigen::Vector2d Vector2d;

    /**@brief type for grid position
     */
    typedef Eigen::Vector3d Vector3d;

    typedef Eigen::AlignedBox<unsigned int, 2> CellExtents;

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    class Index : public Vector2i
    {
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & x();
            ar & y();
        }

        public:
            typedef Vector2i Base;

            Index()
                : Vector2i(0, 0)
            {}

            Index(int x, int y) 
                : Vector2i(x, y)
            {}

            /* @brief this constructor allows you to construct Index from Eigen expressions
             */
            template<typename OtherDerived>
            Index(const Eigen::MatrixBase<OtherDerived>& other)
                : Vector2i(other)
            {}

            /**
             * @brief Check if index is inside the other.
             * @details 
             * Returns true if x and y of index are smaller the x and y of other
             * @return [description]
             */
            bool isInside(const Index& other) const
            {
                return (x() < other.x() && y() < other.y());
            }

            /**
             * @brief Check if index is inside cell size.
             * @details 
             * Returns true if x and y of index are smaller the x and y of other
             * @return [description]
             */
            bool isInside(const Vector2ui& other) const
            {
                return (x() < static_cast<int>(other.x()) && y() < static_cast<int>(other.y())) && (x() >= 0.00 && y() >= 0.00);
            }

    };

    /** @brief Lexicographical ordering (Strict Total Ordering)
     * @details
     * Returns true if:
     *      - index.x() < other.x()
     *      - index.x() == other.x() && index.y() < other.y()
     * Otherwise, return false.
     */
    inline bool operator<(const Index& lhs, const Index& rhs)
    {
        return (lhs.x() < rhs.x()
                || (lhs.x() == rhs.x() && lhs.y() < rhs.y()));
    }

}}

#endif // __MAPS_INDEX_HPP__
