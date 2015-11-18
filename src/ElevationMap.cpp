//#include "ElevationMap.hpp"
#include "GridMap.hpp"

typedef envire::maps::GridMap<double> ElevatioMap;

//namespace envire 
//{
//namespace maps 
//{
//
//const std::string ElevationMap::ELEVATION = "elevation_max"; // this will reference the max band
//const std::string ElevationMap::ELEVATION_MIN = "elevation_min";
//const std::string ElevationMap::ELEVATION_MAX = "elevation_max";
//
//const double ElevationMap::ELEVATION_MIN_DEFAULT = std::numeric_limits<double>::infinity();
//const double ElevationMap::ELEVATION_MAX_DEFAULT = -std::numeric_limits<double>::infinity();
//
//ElevationMap::ElevationMap() 
//    : GridMap() 
//{}
//
//ElevationMap::ElevationMap(GridConfig config)
//    : GridMap(config) 
//{
//    // add all layers to the grid without memory allocation and default value initialization
//    addGrid<double>(ELEVATION_MAX, ELEVATION_MAX_DEFAULT);
//    addGrid<double>(ELEVATION_MIN, ELEVATION_MIN_DEFAULT); 
//}
//
//ElevationMap::~ElevationMap()
//{
//
//}
//
//Eigen::Vector3d ElevationMap::getNormal(const Index& idx) const
//{
//    if (!inGrid(idx))
//        throw std::runtime_error("Provided index is out of grid.");
//    if (!inGrid(idx + Index(1,1)))
//        throw std::runtime_error("Provided index should be at the distance Index(1,1) from the grid border.");
//    if (idx < Index(1,1))
//        throw std::runtime_error("Provided index should be at the distance Index(1,1) from the grid border.");            
//
//    size_t m = idx.x, n = idx.y;
//
//    const Grid<double> &elev_grid = getGrid<double>(ELEVATION);
//
//    double slope_x = (elev_grid.at(m - 1, n) - elev_grid.at(m + 1, n)) / (getScaleX() * 2.0); 
//    double slope_y = (elev_grid.at(m, n - 1) - elev_grid.at(m, n + 1)) / (getScaleY() * 2.0);
//
//    return Eigen::Vector3d( slope_x, slope_y, 1.0 ).normalized();
//}
//
//Eigen::Vector3d ElevationMap::getNormal(const Eigen::Vector2d& pos) const
//{
//    Index idx;
//    if (!toGrid(pos, idx))
//        throw std::runtime_error("Provided position is out of the grid.");
//
//    return getNormal(idx);
//}
//
//double ElevationMap::getMeanElevation(const Eigen::Vector2d& pos) const
//{
//    Index idx;
//    if (!toGrid(pos, idx))
//        throw std::runtime_error("Provided position is out of the grid.");
//
//    Eigen::Vector2d pos_t;
//    fromGrid(idx, pos_t);
//
//    //X and Y in the cell
//    pos_t.x() -= pos.x();
//    pos_t.y() -= pos.y();
//
//    Eigen::Vector3d normal = getNormal( pos );
//    double slope_x = normal.x() / normal.z();
//    double slope_y = normal.y() / normal.z();
//
//    const Grid<double> &elev_grid = getGrid<double>(ELEVATION);
//
//    double height = 
//        elev_grid.at(idx) +
//        pos_t.x() * slope_x +
//        pos_t.y() * slope_y;
//
//    return height;
//}
//
//}
//}
