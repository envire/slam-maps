#ifndef __MAPS_BRESENHAM_LINE__
#define __MAPS_BRESENHAM_LINE__

#include <Eigen/Core>

namespace maps { namespace tools
{


/**
 * This class use useful for calculation
 * straight lines in grid. The Bresenham Algorithm
 * is a fast but inaccurate way to do this. 
 * Inaccurate means, that the result will have
 * aliasing artifacts.
 * 
 * The implementation of the core algorithm was taken from an wikipedia article.
 * */
class Bresenham {
public:
    typedef Eigen::Vector2i Point;
    Bresenham(const Point& start, const Point& end);

    /**
     * Inits the algorithm.
     * The method may be used to 'reinit' the class
     * after a line was already interpolated.
     * */
    void init(const Point& start, const Point& end);
    void init(int startX, int startY, int endX, int endY);

    /**
     * Calculates the next point in the line
     * and returns it over the given parameters.
     *
     * returns false if the end point is reached an
     *              no further point can be calculated.
     *
     * */
    bool getNextPoint(Point& next);

private:
    bool hasP;
    int x0,y0, x1, y1, dx, sx, dy, sy, err;
};


}}  // namespace maps


#endif
