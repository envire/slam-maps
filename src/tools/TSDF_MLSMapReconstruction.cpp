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
#include "TSDF_MLSMapReconstruction.hpp"

using namespace maps::tools;
using namespace maps::grid;

void TSDF_MLSMapReconstruction::reconstruct(MLSMapPrecalculated& output)
{
    std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > surfaces;
    std::vector<float> intensities;
    reconstructSurfaces(surfaces, intensities, false);

    output = MLSMapPrecalculated(tsdf_map->getNumCells() + maps::grid::Vector2ui(1,1), tsdf_map->getResolution(), MLSConfig());
    output.getLocalFrame().translation() << 0.5*tsdf_map->getResolution(), 0;

    Eigen::Vector3f p1, p2, p3;
    Index idx;
    Eigen::Vector3f center;
    Eigen::Vector3f normal;
    float min_z, max_z;
    for(unsigned i = 0; i < surfaces.size(); i += 3)
    {
        p1 = surfaces[i];
        p2 = surfaces[i+1];
        p3 = surfaces[i+2];
        normal = (p2 - p1).cross(p3 - p1);
        normal.normalize();

        // in case the vectors are zero or linearly dependent
        if(!normal.allFinite())
            continue;

        center = (p1 + p2 + p3) / 3.f;
        min_z = std::min(p1.z(), std::min(p2.z(), p3.z()));
        max_z = std::max(p1.z(), std::max(p2.z(), p3.z()));

        Eigen::Vector3d pos_diff;
        if(output.toGrid(center.cast<double>(), idx, pos_diff))
        {
            SurfacePatch< MLSConfig::PRECALCULATED > patch(pos_diff.cast<float>(), normal, min_z, max_z);
            // insert as new patch
            output.at(idx).insert(patch);
        }
        else
        {
            std::stringstream stream;
            stream << center.transpose();
            stream << " is outside of grid! This should never happen.";
            throw std::runtime_error(stream.str());
        }
    }

    // set local frame
    output.getLocalFrame() = output.getLocalFrame() * tsdf_map->getLocalFrame();
}
