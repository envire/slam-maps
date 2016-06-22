#include "ElevationMap.hpp"

namespace maps { namespace grid
{

const float ElevationMap::ELEVATION_DEFAULT = std::numeric_limits<float>::infinity();

ElevationMap::ElevationMap() 
   : GridMapF() 
{}

ElevationMap::ElevationMap(const GridMapF &grid_map) 
    : GridMapF(grid_map)
{}


ElevationMap::ElevationMap(const ElevationMap &elevation_map) 
   : GridMapF(elevation_map) 
{}

ElevationMap::ElevationMap(const Vector2ui &num_cells, const Vector2d &resolution)
   : GridMapF(num_cells, resolution, ELEVATION_DEFAULT)
{}

ElevationMap::ElevationMap(const Vector2ui &num_cells, const Vector2d &resolution, const float &default_value)
   : GridMapF(num_cells, resolution, default_value)
{}

ElevationMap::~ElevationMap()
{}

Vector3d ElevationMap::getNormal(const Index& idx) const
{
    if (!inGrid(idx))
        throw std::runtime_error("Provided index is out of grid.");
    if (!inGrid(idx + Index(1,1)))
        throw std::runtime_error("Provided index should be at the distance Index(1,1) from the grid border.");
    if (idx < Index(1,1))
        throw std::runtime_error("Provided index should be at the distance Index(1,1) from the grid border."); 

    size_t m = idx.x(), n = idx.y();

    // if the neighbour cells have no values
    if (at(m - 1, n) == ELEVATION_DEFAULT
        || at(m + 1, n) == ELEVATION_DEFAULT
        || at(m, n - 1) == ELEVATION_DEFAULT
        || at(m, n + 1) == ELEVATION_DEFAULT)
    {
        return Vector3d(NAN, NAN, NAN);
    }

    float slope_x = (at(m - 1, n) - at(m + 1, n)) / (getResolution().x() * 2.0); 
    float slope_y = (at(m, n - 1) - at(m, n + 1)) / (getResolution().y() * 2.0);

    return Vector3d(slope_x, slope_y, 1.0 ).normalized();
}

Vector3d ElevationMap::getNormal(const Vector3d& pos) const
{
   Index idx;
   if (!toGrid(pos, idx))
       throw std::runtime_error("Provided position is out of the grid.");

   return getNormal(idx);
}

float ElevationMap::getMeanElevation(const Vector3d& pos) const
{
    Index idx;
    if (!toGrid(pos, idx))
        throw std::runtime_error("Provided position is out of the grid.");

    // if there are no value in the cell
    if (at(idx) == ELEVATION_DEFAULT)
        return ELEVATION_DEFAULT;

    Vector3d pos_t(0.00, 0.00, 0.00);
    fromGrid(idx, pos_t);

    //X and Y in the cell
    pos_t.x() -= pos.x();
    pos_t.y() -= pos.y();

    Vector3d normal = getNormal(pos);

    // if there are no values in the neighbours cells
    if (normal.hasNaN()) 
        return ELEVATION_DEFAULT;

    float slope_x = normal.x() / normal.z();
    float slope_y = normal.y() / normal.z();

    float height = 
        at(idx) +
        pos_t.x() * slope_x +
        pos_t.y() * slope_y;

    return height;
}

std::pair<float, float> ElevationMap::getElevationRange() const
{
    float max = -std::numeric_limits<float>::infinity();
    float min = std::numeric_limits<float>::infinity();
    for (unsigned int y = 0; y < getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < getNumCells().x(); ++x)
        {
            if (at(x, y) == ELEVATION_DEFAULT)
                continue;

            if (at(x, y) > max)
                max = at(x, y);

            if (at(x, y) < min)
                min = at(x, y);
        }
    }
    return std::pair<float, float>(min, max);
}

}}
