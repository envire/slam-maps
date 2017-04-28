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
