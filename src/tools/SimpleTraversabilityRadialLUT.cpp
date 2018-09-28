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

#include "SimpleTraversabilityRadialLUT.hpp"

#include <boost/tuple/tuple.hpp>

using namespace maps;
using namespace tools;

void SimpleTraversabilityRadialLUT::precompute(double distance, double resolutionX, double resolutionY)
{
    const double radius2 = distance * distance;

    width  = 2 * ceil(distance / resolutionX) + 1;
    height = 2 * ceil(distance / resolutionY) + 1;

    inDistance.resize(boost::extents[height][width]);
    std::fill(inDistance.data(), inDistance.data() + inDistance.num_elements(), false);

    parents.resize(boost::extents[height][width]);
    std::fill(parents.data(), parents.data() + parents.num_elements(), std::make_pair(-1, -1));

    centerX = width  / 2;
    centerY = height / 2;

    parents[centerY][centerX] = std::make_pair(-1, -1);

    for (unsigned int y = 0; y < height; ++y)
    {
        for (unsigned int x = 0; x < width; ++x)
        {
            int dx = (x - centerX);
            int dy = (y - centerY);

            if (dx == 0 && dy == 0)
                continue;

            double d2 = dx * dx * resolutionX * resolutionX + dy * dy * resolutionY * resolutionY;
            inDistance[y][x] = (d2 < radius2);

            if (abs(dx) > abs(dy))
            {
                int parentx = x - dx / abs(dx);
                int parenty = y - rint(static_cast<double>(dy) / abs(dx));
                parents[y][x] = std::make_pair(parenty, parentx);
            }
            else
            {
                int parentx = x - rint(static_cast<double>(dx) / abs(dy));
                int parenty = y - dy / abs(dy);
                parents[y][x] = std::make_pair(parenty, parentx);
            }
        }
    }
}

void SimpleTraversabilityRadialLUT::markAllRadius(grid::TraversabilityGrid& traversabilityGrid, int centerX, int centerY, int targetedValue) const
{
    int baseX = centerX - this->centerX;
    int baseY = centerY - this->centerY;

    for (unsigned int y = 0; y < height; ++y)
    {
        int mapY = baseY + y;
        if (mapY < 0 || mapY >= static_cast<int>(traversabilityGrid.getNumCells()[1]))
            continue;

        for (unsigned int x = 0; x < width; ++x)
        {
            int mapX = baseX + x;
            if (mapX < 0 || mapX >= static_cast<int>(traversabilityGrid.getNumCells()[0]))
                continue;

            if (inDistance[y][x] && traversabilityGrid.at(grid::Index(mapX, mapY)).getTraversabilityClassId() == targetedValue)
            {
                markSingleRadius(traversabilityGrid, centerX, centerY, x, y, targetedValue, targetedValue);
            }
        }
    }
}

void SimpleTraversabilityRadialLUT::markSingleRadius(grid::TraversabilityGrid& traversabilityGrid, int centerX, int centerY, int targetX, int targetY, int expectedValue, int markValue) const
{
    boost::tie(targetY, targetX) = parents[targetY][targetX];

    while (targetX != -1 && targetY != -1)
    {
        int mapX = centerX + targetX - this->centerX;
        int mapY = centerY + targetY - this->centerY;

        if (mapX < 0 || mapX >= static_cast<int>(traversabilityGrid.getNumCells()[0]) || mapY < 0 || mapY >= static_cast<int>(traversabilityGrid.getNumCells()[1]))
        {
            boost::tie(targetY, targetX) = parents[targetY][targetX];
            continue;
        }

        if (traversabilityGrid.at(grid::Index(mapX, mapY)).getTraversabilityClassId() != expectedValue)
        {
            traversabilityGrid.setTraversabilityAndProbability(markValue, 1, mapX, mapY);
        }

        boost::tie(targetY, targetX) = parents[targetY][targetX];
    }
}
