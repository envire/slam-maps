#include "ElevationMap.hpp"

namespace maps 
{   

const double ElevationData::ELEVATION_DEFAULT = std::numeric_limits<double>::infinity();
const double ElevationData::ELEVATION_MIN_DEFAULT = -std::numeric_limits<double>::infinity();     

ElevationMap::ElevationMap() 
   : GridMap<ElevationData>() 
{}

ElevationMap::ElevationMap(const Vector2ui &num_cells, const Vector2d &resolution)
   : GridMap<ElevationData>(num_cells, resolution, ElevationData()) 
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
    if (at(m - 1, n).elevation == ElevationData::ELEVATION_DEFAULT
        || at(m + 1, n).elevation == ElevationData::ELEVATION_DEFAULT
        || at(m, n - 1).elevation == ElevationData::ELEVATION_DEFAULT
        || at(m, n + 1).elevation == ElevationData::ELEVATION_DEFAULT)
    {
        return Vector3d(NAN, NAN, NAN);
    }

    double slope_x = (at(m - 1, n).elevation - at(m + 1, n).elevation) / (getResolution().x() * 2.0); 
    double slope_y = (at(m, n - 1).elevation - at(m, n + 1).elevation) / (getResolution().y() * 2.0);

    return Vector3d(slope_x, slope_y, 1.0 ).normalized();
}

Vector3d ElevationMap::getNormal(const Vector3d& pos) const
{
   Index idx;
   if (!toGrid(pos, idx))
       throw std::runtime_error("Provided position is out of the grid.");

   return getNormal(idx);
}

double ElevationMap::getMeanElevation(const Vector3d& pos) const
{
    Index idx;
    if (!toGrid(pos, idx))
        throw std::runtime_error("Provided position is out of the grid.");

    // if there are no value in the cell
    if (at(idx).elevation == ElevationData::ELEVATION_DEFAULT)
        return ElevationData::ELEVATION_DEFAULT;

    Vector3d pos_t;
    fromGrid(idx, pos_t);

    //X and Y in the cell
    pos_t.x() -= pos.x();
    pos_t.y() -= pos.y();

    Vector3d normal = getNormal(pos);

    // if there are no values in the neighbours cells
    if (normal.hasNaN()) 
        return ElevationData::ELEVATION_DEFAULT;

    double slope_x = normal.x() / normal.z();
    double slope_y = normal.y() / normal.z();

    double height = 
        at(idx).elevation +
        pos_t.x() * slope_x +
        pos_t.y() * slope_y;

    return height;
}

std::pair<double, double> ElevationMap::getElevationRange() const
{
    double max = -std::numeric_limits<double>::infinity();
    double min = std::numeric_limits<double>::infinity();
    for (unsigned int y = 0; y < getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < getNumCells().x(); ++x)
        {
            if (at(x, y).elevation == ElevationData::ELEVATION_DEFAULT)
                continue;

            if (at(x, y).elevation > max)
                max = at(x, y).elevation;

            if (at(x, y).elevation < min)
                min = at(x, y).elevation;
        }
    }
    return std::pair<double, double>(min, max);
}

std::pair<double, double> ElevationMap::getElevationMinRange() const
{
    double max = -std::numeric_limits<double>::infinity();
    double min = std::numeric_limits<double>::infinity();
    for (unsigned int y = 0; y < getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < getNumCells().x(); ++x)
        {
            if (at(x, y).elevation_min == ElevationData::ELEVATION_MIN_DEFAULT)
                continue;

            if (at(x, y).elevation_min > max)
                max = at(x, y).elevation_min;

            if (at(x, y).elevation_min < min)
                min = at(x, y).elevation_min;
        }
    }
    return std::pair<double, double>(min, max);
}

}
