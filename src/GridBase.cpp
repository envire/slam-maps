#include "GridBase.hpp"

namespace envire
{
namespace maps 
{

GridBase::GridBase()
{}	

GridBase::GridBase(GridConfig config)
    : config(config) 
{}

GridBase::~GridBase() 
{}

/**
* TODO: check in Grid and toGrid calculation of in grid position of the point x, y
*/
bool GridBase::toGrid(double x, double y, size_t& xi, size_t& yi, double& xmod, double& ymod) const
{
    size_t xi_t = floor((x - config.offsetX) / config.scaleX);
    size_t yi_t = floor((y - config.offsetY) / config.scaleY);

    if(inGrid(xi_t, yi_t))
    {
		xi = xi_t;
		yi = yi_t;
		xmod = x - (xi * config.scaleX + config.offsetX);
		ymod = y - (yi * config.scaleY + config.offsetY);
		return true;
    }
    else {
		return false;
    }
}

bool GridBase::toGrid(const Eigen::Vector2d& pos, Index& idx) const
{
    Eigen::Vector2d posDiff;
    return toGrid(pos.x(), pos.y(), idx.x, idx.y, posDiff.x(), posDiff.y());
}

bool GridBase::toGrid(const Eigen::Vector2d& pos, Index &idx, Eigen::Vector2d& posDiff) const
{
    return toGrid(pos.x(), pos.y(), idx.x, idx.y, posDiff.x(), posDiff.y());
}

bool GridBase::fromGrid(size_t xi, size_t yi, double& x, double& y) const
{
    if (inGrid(xi, yi))
    {        
        x = (xi + 0.5) * config.scaleX + config.offsetX;
        y = (yi + 0.5) * config.scaleY + config.offsetY;
        return true;
    }
    else {
        return false;
    }
}

bool GridBase::fromGrid(const Index& idx, Eigen::Vector2d& pos) const
{
    return fromGrid(idx.x, idx.y, pos.x(), pos.y());
}

bool GridBase::inGrid(const Index& idx) const
{
    return inGrid(idx.x, idx.y);
}

bool GridBase::inGrid(size_t xi, size_t yi) const
{
    return (0 <= xi && xi < config.cellSizeX && 0 <= yi && yi < config.cellSizeY);
}

}
}