#include "VoxelTraversal.hpp"

#include <base/Timeout.hpp>
#include <iostream>

using namespace maps::tools;

void VoxelTraversal::computeRay(const Eigen::Vector3d& grid_res, const Eigen::Vector3d& origin, const Eigen::Vector3i& origin_idx, const Eigen::Vector3d& origin_cell_center, const Eigen::Vector3d& measurement, const Eigen::Vector3i& measurement_idx, std::vector< VoxelTraversal::RayElement >& ray)
{
    ray.clear();
    Eigen::Vector3d direction = (measurement - origin).normalized();
    double distance = (measurement - origin).norm();

    Eigen::Vector3i step = Eigen::Vector3i::Zero();
    Eigen::Vector3d t_max = Eigen::Vector3d::Ones() * std::numeric_limits<double>::max();
    Eigen::Vector3d t_delta = Eigen::Vector3d::Zero();

    // compute initial coefficients
    for(unsigned i = 0; i < 3; i++)
    {
        if(direction(i) > 0.0)
            step(i) = 1;
        else if(direction(i) < 0.0)
            step(i) = -1;

        if(step(i) != 0)
        {
            double voxel_border = origin_cell_center(i);
            voxel_border += double(step(i)) * grid_res(i) * 0.5;

            t_max(i) = (voxel_border - origin(i)) / direction(i);
            t_delta(i) = grid_res(i) / fabs(direction(i));
        }
    }

    if(step.x() == 0 && step.y() == 0 && step.z() == 0)
        return;

    Eigen::Vector3i ray_idx = origin_idx;
    ray.push_back(RayElement(ray_idx, step.z()));

    // traverse ray
    while(true)
    {
        // check if we reached the final index
        if(ray_idx == measurement_idx)
            break;

        // check for a potential miss of the last cell due to discretization errors
        if(t_max.minCoeff() > distance)
        {
            // clear ray and return
            ray.clear();
            return;
        }

        // idendify axis to increase
        int axis = 0;
        if(t_max.x() < t_max.y())
            axis = t_max.x() < t_max.z() ? 0 : 2;
        else
            axis = t_max.y() < t_max.z() ? 1 : 2;

        // increase index
        ray_idx[axis] += step[axis];
        t_max[axis] += t_delta[axis];
        if(axis != 2)
            ray.push_back(RayElement(ray_idx, step.z()));
        else
            ray.back().z_last = ray_idx[axis];
    }

    assert(ray_idx.block(0,0,2,1) == ray.back().idx && ray_idx.z() == ray.back().z_last);

    // remove last element from ray
    if(!ray.empty())
    {
        if(ray.back().z_last != ray.back().z_first)
            ray.back().z_last -= step.z();
        else
            ray.pop_back();
    }
}