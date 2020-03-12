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
#define BOOST_TEST_MODULE ToolsTest
#include <boost/test/unit_test.hpp>

#include <maps/grid/VoxelGridMap.hpp>

#include <maps/tools/VoxelTraversal.hpp>
#include <maps/grid/SurfacePatches.hpp>
#include <iostream>


template<class Iterator>
bool checkRayElements(const Iterator& ray_start, const Iterator& ray_end, const Eigen::Vector3i& next_idx)
{
    if(ray_start == ray_end)
        return true;

    if(next_idx.x() != ray_start->x())
        return false;

    if(next_idx.y() != ray_start->y())
        return false;

    if(next_idx.z() != ray_start->z())
        return false;

    if(checkRayElements(ray_start + 1, ray_end,  next_idx + Eigen::Vector3i(1,0,0)) ||
        checkRayElements(ray_start + 1, ray_end, next_idx + Eigen::Vector3i(0,1,0)) ||
        checkRayElements(ray_start + 1, ray_end, next_idx + Eigen::Vector3i(0,0,1)) ||
        checkRayElements(ray_start + 1, ray_end, next_idx + Eigen::Vector3i(-1,0,0)) ||
        checkRayElements(ray_start + 1, ray_end, next_idx + Eigen::Vector3i(0,-1,0)) ||
        checkRayElements(ray_start + 1, ray_end, next_idx + Eigen::Vector3i(0,0,-1)))
    {
        return true;
    }

    std::cerr << "Cant find a following element from " << next_idx << std::endl;

    return false;
}

bool checkRay(const std::vector<Eigen::Vector3i> &ray)
{
    if(ray.empty())
        return false;

    return checkRayElements(ray.begin()+1, ray.end(), *(ray.begin()+1));
}



BOOST_AUTO_TEST_CASE(test_voxel_traversal_continuity)
{
    Eigen::Vector3d resolution(0.1,0.07367,0.05);

    unsigned runs = 0;
    Eigen::Affine3d transformation;
    Eigen::Quaterniond q;
    std::vector<Eigen::Vector3i> ray;
    while(true)
    {
        // generate new transformation
        transformation.translation().setRandom();
        q.coeffs().setRandom();
        q.normalize();
        transformation.linear() = q.toRotationMatrix();

        Eigen::Vector3d origin = transformation.translation();
        Eigen::Vector3d measurement = transformation * (Eigen::Vector3d::UnitX() * 10.0);

        maps::tools::VoxelTraversal::computeRay(resolution, origin, measurement, ray);

        if(ray.empty())
        {
            std::cerr << "Ray is empty, probably missed the last element while ray tracing." << std::endl;
            continue;
        }

        BOOST_CHECK(checkRay(ray));

        runs++;
        if(runs > 1000000)
            break;
    }
}
