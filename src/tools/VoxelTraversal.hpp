#pragma once

#include <Eigen/Core>
#include <maps/grid/Index.hpp>

namespace maps { namespace tools
{

class VoxelTraversal
{
public:

    struct RayElement
    {
        RayElement(const Eigen::Vector3i& idx, int z_step) : idx(idx.x(), idx.y()), z_first(idx.z()), z_last(idx.z()), z_step(z_step) {}

        maps::grid::Index idx;
        int32_t z_first;
        int32_t z_last;
        int32_t z_step;
    };

    /**
     * Computes measurement_idx and origin_cell_center before calling computeRay.
     * If you are unsure about the cell alignment, use this method.
     */
    static void computeRay(const Eigen::Vector3d& grid_res, const Eigen::Vector3d& origin,
                           const Eigen::Vector3i& origin_idx, const Eigen::Vector3d& measurement,
                           std::vector<RayElement>& ray);

    /**
     * Voxel traversal algorithm implemented according to:
     * Amanatides, John, and Andrew Woo. "A fast voxel traversal algorithm for ray tracing." Eurographics. Vol. 87. No. 3. 1987.
     */
    static void computeRay(const Eigen::Vector3d& grid_res,
                           const Eigen::Vector3d& origin, const Eigen::Vector3i& origin_idx, const Eigen::Vector3d& origin_cell_center,
                           const Eigen::Vector3d& measurement, const Eigen::Vector3i& measurement_idx,
                           std::vector<RayElement>& ray);

};



}}