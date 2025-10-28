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
#include "TSDFPolygonMeshReconstruction.hpp"

#include <pcl/common/transforms.h>

using namespace maps::tools;
using namespace maps::grid;

void TSDFPolygonMeshReconstruction::reconstruct(pcl::PolygonMesh& output)
{
    std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > surfaces;
    std::vector<float> intensities;
    reconstructSurfaces(surfaces, intensities);

    pcl::PointCloud<pcl::PointXYZI> cloud_with_intensity;
    cloud_with_intensity.resize(surfaces.size());
    for(unsigned i = 0; i < surfaces.size(); i++)
    {
        cloud_with_intensity[i].getVector3fMap() = surfaces[i];
        cloud_with_intensity[i].intensity = intensities[i];
    }

    pcl::toPCLPointCloud2(cloud_with_intensity, output.cloud);

    output.polygons.resize (cloud_with_intensity.size() / 3);
    for(size_t i = 0; i < output.polygons.size(); ++i)
    {
        pcl::Vertices v;
        v.vertices.resize (3);
        for(int j = 0; j < 3; ++j)
        {
            v.vertices[j] = static_cast<int> (i) * 3 + j;
        }
        output.polygons[i] = v;
    }
}
