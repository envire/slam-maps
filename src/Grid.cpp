#include "Grid.hpp"

namespace envire { namespace maps 
{

Grid::Grid(const Eigen::Vector2d &resolution, const Vector2ui &num_cells)
    : resolution(resolution),
      num_cells(num_cells)
{}

Grid::~Grid()
{}

}}