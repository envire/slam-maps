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
