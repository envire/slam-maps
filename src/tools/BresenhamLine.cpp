#include "BresenhamLine.hpp"

namespace maps { namespace tools
{


Bresenham::Bresenham(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
    init(start.x(), start.y(), end.x(), end.y());
}


void Bresenham::init(const Point& start, const Point& end)
{
    init(start.x(), start.y(), end.x(), end.y());
}

void Bresenham::init(int startX, int startY, int endX, int endY)
{
    x0 = startX;
    y0 = startY;
    x1 = endX;
    y1 = endY;
    dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
    dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1; 
    err = dx+dy; /* error value e_xy */
    hasP = true;
}


bool Bresenham::getNextPoint(Point & next)
{
    if(!hasP)
        return false;
    next << x0, y0;
    
    if (x0==x1 && y0==y1)
    {
        hasP = false;
        return true;
    }
    
    int e2 = 2*err;
    if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
    
    return true;
}



}  // namespace tools
}  // namespace maps
