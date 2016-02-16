#include "Grid.hpp"

#include <iostream>

namespace envire { namespace maps 
{

Grid::Grid()
    : LocalMap(),
      num_cells(0,0),
      resolution(0,0)
{}

Grid::Grid(const Vector2ui &num_cells, const Eigen::Vector2d &resolution)
    : LocalMap(),
      num_cells(num_cells),
      resolution(resolution)
{}

Grid::Grid(const Vector2ui &num_cells, 
            const Vector2d &resolution, 
            const boost::shared_ptr<LocalMapData> &data)
    : LocalMap(data),
      num_cells(num_cells),
      resolution(resolution)
{}

Grid::~Grid()
{}

const Vector2ui& Grid::getNumCells() const
{
    return num_cells;
}

const Vector2d& Grid::getResolution() const 
{ 
    return resolution; 
}

Vector2d Grid::getSize() const
{
    return resolution.array() * num_cells.cast<double>().array();
}

bool Grid::inGrid(const Index& idx) const
{
    // do not need to check idx against (0,0),
    // until idx is of type unsigned int
    return (idx.array() < Index(num_cells).array()).all();
}

bool Grid::fromGrid(const Index& idx, Vector3d& pos) const
{
    /** Index inside the grid **/
    if (inGrid(idx))
    {
        // position at the cell center without offset trnasformation
        Vector2d center = (idx.cast<double>() + Vector2d(0.5, 0.5)).array() * resolution.array();

        // Apply the offset transformation to the obtained position
        pos = this->getOffset() * Vector3d(center.x(), center.y(), 0.);
        return true;
    } else
    {
        /** Index outside the grid **/
        return false;
    }
}

bool Grid::fromGrid(const Index& idx, Vector3d& pos_in_frame, const base::Transform3d &frame_in_grid) const
{
    Vector3d pos_in_map;

    if (fromGrid(idx, pos_in_map) == false)
        return false;

    pos_in_frame = frame_in_grid.inverse() * pos_in_map;

    return true;
}

bool Grid::toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_diff) const
{
    // position without offset
    Vector2d pos_temp = (this->getOffset().inverse() * pos).head<2>();  

    // cast to float due to position which lies on the border between two cells
    Eigen::Vector2d idx_double = pos_temp.array() / resolution.array();
    Index idx_temp(std::floor(idx_double.x()), std::floor(idx_double.y()));

    Vector3d center;
    if(inGrid(idx_temp) && fromGrid(idx_temp, center))
    {
        idx = idx_temp; 
        pos_diff = pos - center;    
        return true;
    }
    else {
        return false;
    }
}

bool Grid::toGrid(const Vector3d& pos_in_frame, Index& idx, const base::Transform3d &frame_in_grid) const
{
    Eigen::Vector3d pos_in_map = frame_in_grid * pos_in_frame;
    Eigen::Vector3d pos_diff;
    return toGrid(pos_in_map, idx, pos_diff);
}

bool Grid::toGrid(const Vector3d& pos, Index& idx) const 
{
    Vector3d pos_diff;
    return toGrid(pos, idx, pos_diff);
}

}}
