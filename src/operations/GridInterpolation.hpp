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
#ifndef __MAPS_GRIDINTERPOLATION_HPP__
#define __MAPS_GRIDINTERPOLATION_HPP__

#include "../GridMap.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

namespace maps { namespace operations
{
    class GridInterpolation
    {
    public:
        GridInterpolation() {};
        ~GridInterpolation() {};

        /**
         * @brief [brief description]
         * @details nore only for doubles
         * (cell_value = Vector3d(x,y,1).dot( A.inverse() * b );)
         * uncertanty for integral types
         * 
         * @param grid [description]
         */
        template <class T, 
                class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
        static void interpolate(GridMap<T> &grid)
        {
            typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
            typedef CGAL::Projection_traits_xy_3<K>  Gt;
            typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;                
            typedef K::Point_3 Point;

            Delaunay dt;

            size_t min_width = grid.getNumCells().x() - 1;
            size_t max_width = 0;
            size_t min_height = grid.getNumCells().y() - 1;
            size_t max_height = 0;   

            T default_value = grid.getDefaultValue();

            for(size_t y = 0; y < grid.getNumCells().y(); ++y)
            {
                for(size_t x = 0; x < grid.getNumCells().x(); ++x)
                {
                    T const &cell_value = grid.at(x, y);
                    if(cell_value != default_value )
                    {
                        Point p(x, y, cell_value);
                        dt.insert( p );
                            
                        min_width = std::min(min_width, x);
                        max_width = std::max(max_width, x);
                        min_height = std::min(min_height, y);
                        max_height = std::max(max_height, y);
                    }
                }
            }

            for(size_t x = min_width; x < max_width; x++)
            {
                for(size_t y = min_height; y < max_height; y++)
                {
                    T &cell_value = grid.at(x, y);
                    if(cell_value == default_value)
                    {
                        // no data point in grid, so value needs to be interpolated

                        // Solve linear equation system to find plane that is spanned by
                        // the three points
                        Eigen::Matrix3d A;
                        Eigen::Vector3d b;

                        Delaunay::Face_handle face = dt.locate(Point(x, y, 0));
                        if( face == NULL || face->vertex(0) == NULL 
                            || face->vertex(1) == NULL || face->vertex(2) == NULL)
                            continue;

                        for(int i = 0; i < 3; i++)
                        {
                            Point &p(face->vertex(i)->point());
                            A.block<1,3>(i,0) = Vector3d(p.x(), p.y(), 1);
                            b(i) = p.z();
                        }

                        // evaluate the point at x, y
                        cell_value = Vector3d(x,y,1).dot( A.inverse() * b );
                    }
                }
            }       
        }

    private:

    };
}}

#endif // __MAPS_GRIDINTERPOLATION_HPP__
