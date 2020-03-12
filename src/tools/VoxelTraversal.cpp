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
#include "VoxelTraversal.hpp"

using namespace maps::tools;

void VoxelTraversal::computeRay(const Eigen::Vector3d& grid_res, const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& measurement, std::vector<Eigen::Vector3i>& voxel_indices)
{
    voxel_indices.clear();
    Eigen::Vector3i current_voxel, last_voxel, diff;
    Eigen::Vector3d ray, step, voxel_border, t_max, t_delta;
    diff = Eigen::Vector3i::Zero();
    bool neg_ray = false;

    for(unsigned i = 0; i < 3; i++)
    {
        current_voxel(i) = std::floor(origin(i) / grid_res(i));
        last_voxel(i) = std::floor(measurement(i) / grid_res(i));
        ray(i) = measurement(i) - origin(i);

        // compute initial coefficients
        step(i) = (ray(i) >= 0.) ? 1. : -1.;
        voxel_border(i) = (current_voxel(i) + step(i)) * grid_res(i);
        t_max(i) = (ray(i) != 0.) ? (voxel_border(i) - origin(i)) / ray(i) : std::numeric_limits< double >::max();
        t_delta(i) = (ray(i) != 0.) ? grid_res(i) / ray(i) * step(i) : std::numeric_limits< double >::max();

        if(current_voxel(i) != last_voxel(i) && ray(i) < 0.)
        {
            diff(i)--;
            neg_ray = true;
        }
    }

    voxel_indices.push_back(current_voxel);
    if (neg_ray)
    {
        current_voxel += diff;
        voxel_indices.push_back(current_voxel);
    }

    // traverse ray
    while(last_voxel != current_voxel)
    {
        // identify axis to increase
        int axis = 0;
        if(t_max.x() < t_max.y())
            axis = t_max.x() < t_max.z() ? 0 : 2;
        else
            axis = t_max.y() < t_max.z() ? 1 : 2;

        // increase index
        current_voxel[axis] += step[axis];
        t_max[axis] += t_delta[axis];
        voxel_indices.push_back(current_voxel);
    }
}

void VoxelTraversal::computeRay(const Eigen::Vector3d& grid_res, const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& measurement, std::vector< RayElement >& ray)
{
    std::vector<Eigen::Vector3i> visited_voxels;
    computeRay(grid_res, origin, measurement, visited_voxels);

    ray.clear();
    for(unsigned i = 0; i < visited_voxels.size(); i++)
    {
        if(!ray.empty() && ray.back().idx == visited_voxels[i].head<2>())
        {
            ray.back().z_step = visited_voxels[i].z() - ray.back().z_last;
            ray.back().z_last = visited_voxels[i].z();
        }
        else
        {
            ray.push_back(RayElement(visited_voxels[i], 0));
        }
    }
}