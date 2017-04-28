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

#include <Eigen/Core>
#include <maps/grid/Index.hpp>
#include <vector>

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
