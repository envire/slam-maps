#include "TSDFVolumetricMap.hpp"
#include <boost/format.hpp>
#include <maps/tools/VoxelTraversal.hpp>

using namespace maps::grid;
using namespace maps::tools;

void TSDFVolumetricMap::mergePointCloud(const TSDFVolumetricMap::PointCloud& pc, const base::Transform3d& pc2grid, double measurement_variance)
{
    Eigen::Vector3d sensor_origin = pc.sensor_origin_.block(0,0,3,1).cast<double>();
    Eigen::Vector3d sensor_origin_in_grid = pc2grid * sensor_origin;

    // TODO add transformation uncertainty

    for(PointCloud::const_iterator it=pc.begin(); it != pc.end(); ++it)
    {
        try
        {
            Eigen::Vector3d measurement = it->getArray3fMap().cast<double>();
            mergePoint(sensor_origin_in_grid, pc2grid * measurement, measurement_variance);
        }
        catch(const std::runtime_error& e)
        {
            // TODO use glog or base log for all out prints of this library
            std::cerr << e.what() << std::endl;
        }
    }
}

void TSDFVolumetricMap::mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement, double measurement_variance)
{
    Eigen::Vector3d truncated_direction = (measurement - sensor_origin).normalized();
    truncated_direction = truncation * truncated_direction;
    double ray_length = (measurement - sensor_origin).norm();
    Eigen::Vector3d start_point = sensor_origin;
    Eigen::Vector3d end_point = measurement + truncated_direction;

    Eigen::Vector3i start_point_idx;
    Eigen::Vector3i end_point_idx;
    Eigen::Vector3d start_point_cell_center;
    if(VoxelGridBase::toVoxelGrid(start_point, start_point_idx) &&
        VoxelGridBase::fromVoxelGrid(start_point_idx, start_point_cell_center) &&
        VoxelGridBase::toVoxelGrid(end_point, end_point_idx, false))
    {
        std::vector<VoxelTraversal::RayElement> ray;
        VoxelTraversal::computeRay(VoxelGridBase::getVoxelResolution(), start_point, start_point_idx, start_point_cell_center, end_point, end_point_idx, ray);
        const float res_sigma = 2.f * VoxelGridBase::getVoxelResolution().squaredNorm() / (5.2f*5.2f);
        const float res_sigma_inv = 1.f / res_sigma;
        if(ray.empty())
            throw std::runtime_error("Ray is empty!");

        // re-add last cell in ray
        ray.push_back(VoxelTraversal::RayElement(end_point_idx, 1));

        for(const VoxelTraversal::RayElement& element : ray)
        {
            try
            {
                DiscreteTree<VoxelCellType>& tree = GridMapBase::at(element.idx);
                Eigen::Vector3d cell_center;
                if(GridMapBase::fromGrid(element.idx, cell_center))
                {
                    int32_t z_end = element.z_last + element.z_step;
                    for(int32_t z_idx = element.z_first; z_idx != z_end; z_idx += element.z_step)
                    {
                        cell_center.z() = tree.getCellCenter(z_idx);

                        // compute point on ray closest to the current cell center
                        Eigen::Hyperplane<double, 3> plane(truncated_direction, cell_center);
                        Eigen::Vector3d point_on_ray = plane.projection(sensor_origin);

                        // weight the current measurement according to the distance to the cell center with the inverse normal distribution
                        float phi = std::exp(-(point_on_ray - cell_center).squaredNorm() * res_sigma_inv);
                        if(phi > 0.f)
                            tree.getCellAt(z_idx).update(ray_length - (point_on_ray - sensor_origin).norm(), (1.f/phi) * measurement_variance, truncation, min_variance);
                    }
                }
                else
                {
                    std::cerr << "Failed to receive cell center of " << element.idx << " from grid." << std::endl;
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
