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

    if(next_idx.x() != ray_start->idx.x())
        return false;

    if(next_idx.y() != ray_start->idx.y())
        return false;

    if(next_idx.z() != ray_start->z_first)
        return false;

    Eigen::Vector3i new_idx(next_idx.x(), next_idx.y(), ray_start->z_last);
    if(checkRayElements(ray_start + 1, ray_end,  new_idx + Eigen::Vector3i(1,0,0)) ||
        checkRayElements(ray_start + 1, ray_end, new_idx + Eigen::Vector3i(0,1,0)) ||
        checkRayElements(ray_start + 1, ray_end, new_idx + Eigen::Vector3i(-1,0,0)) ||
        checkRayElements(ray_start + 1, ray_end, new_idx + Eigen::Vector3i(0,-1,0)))
    {
        return true;
    }

    std::cerr << "Cant find a following element from " << next_idx << std::endl;

    return false;
}

bool checkRay(const std::vector<maps::tools::VoxelTraversal::RayElement>& ray)
{
    if(ray.empty())
        return false;

    Eigen::Vector3i idx(ray.front().idx.x(), ray.front().idx.y(), ray.front().z_first);
    return checkRayElements(ray.begin(), ray.end(), idx);
}



BOOST_AUTO_TEST_CASE(test_voxel_traversal_continuity)
{
    Eigen::Vector3d resolution(0.1,0.07367,0.05);
    maps::grid::VoxelGridMap<bool> voxel_grid(maps::grid::Vector2ui(100. / resolution.x(), 100. / resolution.y()), resolution);
    voxel_grid.getLocalFrame().translation() << 0.5*voxel_grid.getSize(), 0;

    unsigned runs = 0;
    Eigen::Affine3d transformation;
    Eigen::Quaterniond q;
    std::vector<maps::tools::VoxelTraversal::RayElement> ray;
    while(true)
    {
        // generate new transformation
        transformation.translation().setRandom();
        q.coeffs().setRandom();
        q.normalize();
        transformation.linear() = q.toRotationMatrix();

        Eigen::Vector3d origin = transformation.translation();
        Eigen::Vector3i origin_idx;
        BOOST_CHECK(voxel_grid.toVoxelGrid(origin, origin_idx));

        Eigen::Vector3d measurement = transformation * (Eigen::Vector3d::UnitX() * 10.0);
        Eigen::Vector3i measurement_idx;
        BOOST_CHECK(voxel_grid.toVoxelGrid(measurement, measurement_idx));

        Eigen::Vector3d origin_center;
        BOOST_CHECK(voxel_grid.fromVoxelGrid(origin_idx, origin_center));

        maps::tools::VoxelTraversal::computeRay(resolution, origin, origin_idx, origin_center, measurement, measurement_idx, ray);

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
