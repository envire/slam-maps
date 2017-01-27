#pragma once

#include <Eigen/Core>
#include <maps/grid/TSDFVolumetricMap.hpp>
#include "MarchingCubes.hpp"

namespace maps { namespace tools
{

template<class T>
class TSDFSurfaceReconstruction
{
public:
    TSDFSurfaceReconstruction() : std_threshold(1.f), iso_level(0.f) {}
    virtual ~TSDFSurfaceReconstruction() {}

    void setTSDFMap(grid::TSDFVolumetricMap::Ptr map, float z_min = -50.f, float z_max = 50.f)
    {
        tsdf_map = map;

        Eigen::Vector3d voxel_res = tsdf_map->getVoxelResolution();
        maps::grid::Vector2ui cells = tsdf_map->getNumCells();

        z_idx_min = (int32_t)std::floor(z_min / voxel_res.z());
        z_idx_max = (int32_t)std::floor(z_max / voxel_res.z());

        Eigen::Vector2d diff_d = cells.cast<double>();
        Eigen::Vector3i res;
        Eigen::Vector3f max_p;
        res << cells.cast<int>(), z_idx_max - z_idx_min;
        max_p << diff_d.x()*voxel_res.x(), diff_d.y()*voxel_res.y(), z_max - z_min;

        grid_scale = res.cast<float>().cwiseInverse().cwiseProduct(max_p);

        // create cell verticies
        vertices.resize(8);
        for(unsigned i = 0; i < 8; ++i)
        {
            vertices[i] = Eigen::Vector3f::Zero();
            if(i & 0x4)
                vertices[i][1] = max_p[1] / float(res[1]);

            if(i & 0x2)
                vertices[i][2] = max_p[2] / float(res[2]);

            if((i & 0x1) ^ ((i >> 1) & 0x1))
                vertices[i][0] = max_p[0] / float(res[0]);
        }
    }

    inline void setIsoLevel(float iso_level) { this->iso_level = iso_level; }
    inline float getIsoLevel() { return this->iso_level; }

    inline void setStdThreshold(float threshold) { this->std_threshold = threshold; }
    inline float getStdThreshold() { return this->std_threshold; }

    virtual void reconstruct(T &output) = 0;

protected:
    void reconstructSurfaces(std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& surfaces, std::vector<float>& intensities, bool surfaces_in_global_frame = true)
    {
        if(!tsdf_map)
            throw std::runtime_error("TSDF map is not set!");

        maps::grid::CellExtents extends = tsdf_map->calculateCellExtents();
        for(unsigned y = extends.min().y() + 1; y < extends.max().y(); y++)
        {
            for(unsigned x = extends.min().x() + 1; x < extends.max().x(); x++)
            {
                const maps::grid::TSDFVolumetricMap::GridMapBase::CellType& tree = tsdf_map->at(x,y);
                for(maps::grid::TSDFVolumetricMap::GridMapBase::CellType::const_iterator cell = tree.begin(); cell != tree.end(); cell++)
                {
                    if(cell->first > z_idx_min && cell->first < z_idx_max)
                    {
                        Eigen::Vector3i idx(x,y,cell->first);
                        reconstructVoxel(cell->second, idx, surfaces, intensities);
                    }
                }
            }
        }

        // transform points to global frame
        if(surfaces_in_global_frame)
        {
            Eigen::Affine3f local_frame = tsdf_map->getLocalFrame().inverse().cast<float>();
            for(Eigen::Vector3f& point : surfaces)
                point = local_frame * point;
        }
    }

    void reconstructVoxel(const grid::TSDFVolumetricMap::VoxelCellType& voxel, Eigen::Vector3i& idx,
                          std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& surfaces, std::vector<float>& intensities)
    {
        if(std::abs(voxel.getDistance()) < tsdf_map->getTruncation() && voxel.getStandardDeviation() < std_threshold)
        {
            std::vector<float> leaf_node(8, 0.f);
            leaf_node[0] = voxel.getDistance();
            if(getValidNeighborList(leaf_node, idx))
            {
                size_t size = surfaces.size();
                // create surface
                MarchingCubes::computeSurfaces(vertices, leaf_node, surfaces, iso_level);

                // move surfaces to the current cell
                Eigen::Vector3f cell_center = grid_scale.cwiseProduct(idx.cast<float>());
                for(size_t i = size; i < surfaces.size(); i++)
                    surfaces[i] = cell_center + surfaces[i];

                // add intensity information
                size_t new_points = surfaces.size() - size;
                float intensity = std::max( 0.f, (std_threshold - voxel.getStandardDeviation() - std::sqrt(tsdf_map->getMinVariance())) / std_threshold);
                for(size_t i = 0; i < new_points; i++)
                {
                    intensities.push_back(intensity);
                }
            }
        }
    }

private:
    bool getGridValue(Eigen::Vector3i pos, float &distance)
    {
        if(!tsdf_map->hasVoxelCell(pos))
            return false;

        const maps::grid::TSDFPatch& cell = tsdf_map->getVoxelCell(pos);

        if(cell.getStandardDeviation() >= std_threshold)
            return false;

        distance = cell.getDistance();
        return true;
    }

    bool getValidNeighborList(std::vector<float> &leaf, const Eigen::Vector3i &index3d)
    {
        if(!getGridValue(index3d + Eigen::Vector3i (1, 0, 0), leaf[1]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (1, 0, 1), leaf[2]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (0, 0, 1), leaf[3]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (0, 1, 0), leaf[4]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (1, 1, 0), leaf[5]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (1, 1, 1), leaf[6]))
            return false;
        if(!getGridValue(index3d + Eigen::Vector3i (0, 1, 1), leaf[7]))
            return false;
        return true;
    }

protected:
    grid::TSDFVolumetricMap::Ptr tsdf_map;

private:
    float std_threshold;
    float iso_level;
    int32_t z_idx_min;
    int32_t z_idx_max;
    Eigen::Vector3f grid_scale;
    std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vertices;
};

}}