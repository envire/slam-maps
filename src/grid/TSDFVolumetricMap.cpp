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
#include "TSDFVolumetricMap.hpp"
#include <boost/format.hpp>
#include <maps/tools/VoxelTraversal.hpp>

using namespace maps::grid;
using namespace maps::tools;

void TSDFVolumetricMap::mergePointCloud(const TSDFVolumetricMap::PointCloud& pc, const base::Transform3d& pc2grid, double measurement_variance)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.head<3>().cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin;

    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        try
        {
            Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
            mergePoint(sensor_origin_in_grid, pc2grid * measurement, measurement_variance);
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << e.what();
        }
    }
}

void TSDFVolumetricMap::mergePointCloud(const TSDFVolumetricMap::PointCloud& pc, const base::TransformWithCovariance& pc2grid, double measurement_variance)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.head<3>().cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid.getTransform() * sensor_origin;

    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> measurement_in_map = pc2grid.composePointWithCovariance(it->getArray3fMap().cast<double>(), Eigen::Matrix3d::Zero());
        try
        {
            Eigen::Vector3d measurement_normal = (measurement_in_map.first - sensor_origin_in_grid).normalized();
            double pose_variance = measurement_normal.transpose() * measurement_in_map.second * measurement_normal;

            mergePoint(sensor_origin_in_grid, measurement_in_map.first, measurement_variance + (std::isfinite(pose_variance) ? pose_variance : 0.));
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << e.what();
        }
    }
}

void TSDFVolumetricMap::mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement, double measurement_variance)
{
    Eigen::Vector3d measurement_normal = (measurement - sensor_origin).normalized();
    Eigen::Vector3d truncated_direction = truncation * measurement_normal;
    double ray_length = (measurement - sensor_origin).norm();
    Eigen::Vector3d start_point = sensor_origin;
    Eigen::Vector3d end_point = measurement + truncated_direction;

    Eigen::Vector3i start_point_idx;
    if(VoxelGridBase::toVoxelGrid(start_point, start_point_idx))
    {
        std::vector<VoxelTraversal::RayElement> ray;
        VoxelTraversal::computeRay(VoxelGridBase::getVoxelResolution(), getLocalFrame() * start_point, getLocalFrame() * end_point, ray);

        if(ray.empty())
            throw std::runtime_error("Ray is empty!");

        const float res_sigma = 2.f * VoxelGridBase::getVoxelResolution().squaredNorm() / (5.2f*5.2f);
        const float res_sigma_inv = 1.f / res_sigma;

        for(const VoxelTraversal::RayElement& element : ray)
        {
            try
            {
                DiscreteTree<VoxelCellType>& tree = GridMapBase::at(element.idx);
                Eigen::Vector3d cell_center;
                if(GridMapBase::fromGrid(element.idx, cell_center))
                {
                    if(element.z_first == element.z_last)
                    {
                        cell_center.z() = tree.getCellCenter(element.z_first);

                        // compute point on ray closest to the current cell center
                        Eigen::Hyperplane<double, 3> plane(measurement_normal, cell_center);
                        Eigen::Vector3d point_on_ray = plane.projection(sensor_origin);

                        // weight the current measurement according to the distance to the cell center with the inverse normal distribution
                        float phi = std::exp(-(point_on_ray - cell_center).squaredNorm() * res_sigma_inv);
                        if(phi > 0.f)
                            tree.getCellAt(element.z_first).update(ray_length - (point_on_ray - sensor_origin).norm(), (1.f/phi) * measurement_variance, truncation, min_variance);
                    }
                    else
                    {
                        int32_t z_end = element.z_last + element.z_step;
                        for(int32_t z_idx = element.z_first; z_idx != z_end; z_idx += element.z_step)
                        {
                            cell_center.z() = tree.getCellCenter(z_idx);

                            // compute point on ray closest to the current cell center
                            Eigen::Hyperplane<double, 3> plane(measurement_normal, cell_center);
                            Eigen::Vector3d point_on_ray = plane.projection(sensor_origin);

                            // weight the current measurement according to the distance to the cell center with the inverse normal distribution
                            float phi = std::exp(-(point_on_ray - cell_center).squaredNorm() * res_sigma_inv);
                            if(phi > 0.f)
                                tree.getCellAt(z_idx).update(ray_length - (point_on_ray - sensor_origin).norm(), (1.f/phi) * measurement_variance, truncation, min_variance);
                        }
                    }
                }
                else
                {
                    LOG_ERROR_S << "Failed to receive cell center of " << element.idx << " from grid.";
                }
            }
            catch(const std::runtime_error& e)
            {
                // rest of the ray is probably out of grid
                break;
            }
        }
    }
    else
        throw std::runtime_error((boost::format("Sensor origin %1% is outside of the grid! Can't add measurement to grid.") % start_point.transpose()).str());
}

bool TSDFVolumetricMap::hasSameFrame(const base::Transform3d& local_frame, const Vector2ui& num_cells, const Vector2d& resolution) const
{
     if(getResolution() == resolution && getNumCells() == num_cells && getLocalFrame().isApprox(local_frame))
         return true;
     return false;
}

float TSDFVolumetricMap::getTruncation()
{
    return truncation;
}

void TSDFVolumetricMap::setTruncation(float truncation)
{
    this->truncation = truncation;
}

float TSDFVolumetricMap::getMinVariance()
{
    return min_variance;
}

void TSDFVolumetricMap::setMinVariance(float min_varaince)
{
    this->min_variance = min_varaince;
}
