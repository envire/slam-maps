#include "TSDFPolygonMeshReconstruction.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

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