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
#pragma once

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace maps { namespace tools {

class SurfaceIntersection
{
public:

/**
 * Computes the intersection points between a plane and an axis aligned box.
 * Note: This method doesn't check if the plane intersects with the box.
 */
template<class Scalar, int MatrixOptions, class Allocator>
static void computeIntersections(const Eigen::Hyperplane<Scalar, 3>& plane, const Eigen::AlignedBox<Scalar, 3>& box, std::vector< Eigen::Matrix<Scalar, 3, 1, MatrixOptions>, Allocator >& intersections)
{
    typedef Eigen::Matrix<Scalar, 3, 1, MatrixOptions> Vec;
    Vec normal = plane.normal();
    Vec box_center = box.center();
    Vec extents = box.max() - box_center;
    const Scalar dist = plane.signedDistance(box_center);

    // find the max coefficient of the normal:
    int i=0, j=1, k=2;
    if(std::abs(normal[i]) < std::abs(normal[j])) std::swap(i,j);
    if(std::abs(normal[i]) < std::abs(normal[k])) std::swap(i,k);

    const Scalar dotj = extents[j] * normal[j];
    const Scalar dotk = extents[k] * normal[k];

    Vec prev_p;
    enum { NONE, LOW, BOX, HIGH } prev_pos = NONE, pos;
    // calculate intersections in direction i:
    for(int n=0; n<5; ++n)
    {
        Vec p(0,0,0);
        Scalar dotp = Scalar(0.0);
        if((n+1)&2)
            dotp += dotj, p[j] = extents[j];
        else
            dotp -= dotj, p[j] = -extents[j];
        if(n&2)
            dotp += dotk, p[k] = extents[k];
        else
            dotp -= dotk, p[k] = -extents[k];

        p[i] = (dist - dotp) / normal[i];

        if( p[i] < -extents[i])
            pos = LOW;
        else if( p[i] > extents[i])
            pos = HIGH;
        else
            pos = BOX;

        if( (prev_pos == LOW || prev_pos == HIGH) && pos != prev_pos )
        {
            // clipping in
            const Scalar h = prev_pos == LOW ? -extents[i] : extents[i];
            const Scalar s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            intersections.push_back(box_center + prev_p + (p - prev_p) * s);
        }
        if( pos == BOX && n!=4 )
        {
            intersections.push_back(box_center + p);
        }
        else if( pos != prev_pos && prev_pos != NONE )
        {
            // clipping out
            const Scalar h = pos == LOW ? -extents[i] : extents[i];
            const Scalar s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            intersections.push_back(box_center + prev_p + (p - prev_p) * s);
        }

        prev_pos = pos;
        prev_p = p;
    }
}
};

}}
