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
    Vec point_on_plane = plane.projection(box_center);
    Vec extents = box.max() - box_center;
    float dist = (point_on_plane - box_center).dot(normal);

    // find the max coefficient of the normal:
    int i=0, j=1, k=2;
    if(std::abs(normal[i]) < std::abs(normal[j])) std::swap(i,j);
    if(std::abs(normal[i]) < std::abs(normal[k])) std::swap(i,k);

    float dotj = extents[j] * normal[j];
    float dotk = extents[k] * normal[k];

    Vec prev_p;
    enum { NONE, LOW, BOX, HIGH } prev_pos = NONE, pos;
    // calculate intersections in direction i:
    for(int n=0; n<5; ++n)
    {
        Vec p(0,0,0);
        float dotp = 0.0f;
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
            float h = prev_pos == LOW ? -extents[i] : extents[i];
            float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            intersections.push_back(box_center + prev_p + (p - prev_p) * s);
        }
        if( pos == BOX && n!=4 )
        {
            intersections.push_back(box_center + p);
        }
        else if( pos != prev_pos && prev_pos != NONE )
        {
            // clipping out
            float h = pos == LOW ? -extents[i] : extents[i];
            float s = (h - prev_p[i]) / (p[i] - prev_p[i]);
            intersections.push_back(box_center + prev_p + (p - prev_p) * s);
        }

        prev_pos = pos;
        prev_p = p;
    }
}
};

}}