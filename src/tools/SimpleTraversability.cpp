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
#include "SimpleTraversability.hpp"
#include "SimpleTraversabilityRadialLUT.hpp"

using namespace maps;
using namespace tools;

SimpleTraversability::SimpleTraversability()
{
}

SimpleTraversability::SimpleTraversability(
        const SimpleTraversabilityConfig& config)
    : config(config)
{
}

SimpleTraversability::SimpleTraversability(
        double maximumSlope,
        unsigned int classCount,
        double groundClearance,
        double minPassageWidth,
        double obstacleClearance)
{
    config.maximumSlope      = maximumSlope;
    config.classCount        = classCount;
    config.groundClearance   = groundClearance;
    config.minPassageWidth   = minPassageWidth;
    config.obstacleClearance = obstacleClearance;
}

void SimpleTraversability::setConfig(const SimpleTraversabilityConfig& config)
{
    this->config = config;
}

void SimpleTraversability::setConfig(double maximumSlope, unsigned int classCount, double groundClearance, double minPassageWidth, double obstacleClearance)
{
    config.maximumSlope = maximumSlope;
    config.classCount = classCount;
    config.groundClearance = groundClearance;
    config.minPassageWidth = minPassageWidth;
    config.obstacleClearance = obstacleClearance;
}

const SimpleTraversabilityConfig& SimpleTraversability::getConfig() const
{
    return config;
}


bool SimpleTraversability::calculateTraversability(grid::TraversabilityGrid& traversabilityOut, const grid::GridMapF& slopesIn, const grid::GridMapF& maxStepsIn) const
{
    // Check for equal input map sizes and resolutions.
    if (maxStepsIn.getNumCells() != slopesIn.getNumCells() || maxStepsIn.getResolution() != slopesIn.getResolution())
        return false;

    // Init TraversabilityGrid with traversabilityClassId = 0 and probability = 0.
    traversabilityOut = grid::TraversabilityGrid(slopesIn.getNumCells(), slopesIn.getResolution(), grid::TraversabilityCell(0, 0));

    int width = traversabilityOut.getNumCells()[0], height = traversabilityOut.getNumCells()[1];
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // Read the values for this cell.
            bool hasSlope     = false,
                 hasMaxStep   = false;

            float slope = slopesIn.at(grid::Index(x, y));
            if (!slopesIn.isDefault(slope))
                hasSlope = true;

            float maxStep =  maxStepsIn.at(grid::Index(x, y));
            if (!maxStepsIn.isDefault(maxStep))
                hasMaxStep = true;

            // First, max_step is an ON/OFF threshold on the groundClearance parameter.
            if (hasMaxStep && config.groundClearance && (fabs(maxStep) > config.groundClearance))
            {
                traversabilityOut.setTraversabilityAndProbability(CLASS_OBSTACLE, 1, x, y);
                continue;
            }

            if (!hasSlope)
            {
                traversabilityOut.setTraversability(CLASS_UNKNOWN, x, y);
                continue;
            }

            if (hasSlope && config.maximumSlope)
            {
                const float absSlope(fabs(slope));
                if(absSlope > config.maximumSlope)
                {
                    traversabilityOut.setTraversabilityAndProbability(CLASS_OBSTACLE, 1, x, y);
                    continue;
                }
                else
                {
                    // Scale current slope to custom classes (antiproportional).
                    int customClassIdx = (config.classCount - 1) - rint(absSlope / config.maximumSlope * (config.classCount - 1));
                    traversabilityOut.setTraversabilityAndProbability(CUSTOM_CLASSES + customClassIdx, 1, x, y);
                }
            }
        }
    }

    // Perform some post processing if required.
    if (config.minPassageWidth > 0)
    {
        closeNarrowPassages(traversabilityOut, config.minPassageWidth);
    }

    if (config.obstacleClearance > 0)
    {
        growObstacles(traversabilityOut, config.obstacleClearance);
    }

    // Registers TraversabilityClasses in the TraversabilityGrid.
    traversabilityOut.setTraversabilityClass(CLASS_OBSTACLE, grid::TraversabilityClass(0));

    float driveability = 0.0;
    for (int i = 0; i < config.classCount; ++i)
    {
        driveability = (1.0 / config.classCount) * (i + 1);
        traversabilityOut.setTraversabilityClass(CUSTOM_CLASSES + i, grid::TraversabilityClass(driveability));
    }

    // Calculates the mean traversability class of the current map ignoring unknown areas.
    uint8_t classId = 0;
    double summedClassIds = 0.0;
    double counter = 0.0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            classId = traversabilityOut.getTraversabilityClassId(x, y);

            // Sum up if not unknown.
            if(classId != CLASS_UNKNOWN) {
                summedClassIds += classId;
                counter++;
            }
        }
    }

    double medianDriveability = 0.55;
    uint8_t meanClassId = counter == 0 ? 0 : summedClassIds / counter + 0.5;

    // If the map is completely unkown or full with obstacles,
    // set the driveability of unknown areas to the median driveability (0.55).
    if(meanClassId <= 1) {
        traversabilityOut.setTraversabilityClass(CLASS_UNKNOWN, grid::TraversabilityClass(medianDriveability));
    } else {
        grid::TraversabilityClass meanTravClass = traversabilityOut.getTraversabilityClass(meanClassId);
        traversabilityOut.setTraversabilityClass(CLASS_UNKNOWN, meanTravClass);
    }

    return true;
}

void SimpleTraversability::closeNarrowPassages(grid::TraversabilityGrid& traversabilityGrid, double minPassageWidth) const
{
    SimpleTraversabilityRadialLUT lut;
    lut.precompute(minPassageWidth, traversabilityGrid.getResolution()[0], traversabilityGrid.getResolution()[1]);

    size_t mapWidth = traversabilityGrid.getNumCells()[0];
    size_t mapHeight = traversabilityGrid.getNumCells()[1];

    grid::TraversabilityGrid traversabilityGridTmp = traversabilityGrid;

    for (size_t y = 0; y < mapHeight; ++y)
    {
        for (size_t x = 0; x < mapWidth; ++x)
        {
            int traversabilityClassId = traversabilityGridTmp.at(grid::Index(x, y)).getTraversabilityClassId();
            if (traversabilityClassId == CLASS_OBSTACLE)
            {
                lut.markAllRadius(traversabilityGrid, x, y, CLASS_OBSTACLE);
            }
        }
    }
}

void SimpleTraversability::growObstacles(grid::TraversabilityGrid& traversabilityGrid, double growthRadius) const
{
    const double growthRadius_squared = pow(growthRadius,2);

    const double resolutionX = traversabilityGrid.getResolution()[0],
                 resolutionY = traversabilityGrid.getResolution()[1];

    const int growthRadius_cellCountX = growthRadius / resolutionX,
              growthRadius_cellCountY = growthRadius / resolutionY;

    grid::TraversabilityGrid traversabilityGridTmp = traversabilityGrid;

    for (unsigned int y = 0; y < traversabilityGrid.getNumCells()[1]; ++y)
    {
        for (unsigned int x = 0; x < traversabilityGrid.getNumCells()[0]; ++x)
        {
            int classId = traversabilityGridTmp.at(grid::Index(x, y)).getTraversabilityClassId();
            if (classId == CLASS_OBSTACLE)
            {
                // Make everything within obstacleClearance range around the obstacle also
                // an obstacle.
                for (int oy = -growthRadius_cellCountY; oy <= growthRadius_cellCountY; ++oy)
                {
                    for (int ox = -growthRadius_cellCountX; ox <= growthRadius_cellCountX; ++ox)
                    {
                        const int tx = x + ox;
                        const int ty = y + oy;
                        if ((pow(ox * resolutionX, 2) + pow(oy * resolutionY, 2) < growthRadius_squared)
                            && tx >= 0 && tx < static_cast<int>(traversabilityGridTmp.getNumCells()[0])
                            && ty >= 0 && ty < static_cast<int>(traversabilityGridTmp.getNumCells()[1]))
                        {
                            traversabilityGrid.setTraversabilityAndProbability(CLASS_OBSTACLE, 1, tx, ty);
                        }
                    }
                }
            }
        }
    }
}
