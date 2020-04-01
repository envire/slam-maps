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

#include <base-logging/Logging.hpp>

#include <maps/tools/TraversabilityGrassfire.hpp>

using namespace maps;
using namespace tools;

TraversabilityGrassfire::TraversabilityGrassfire()
{
}

TraversabilityGrassfire::TraversabilityGrassfire(const TraversabilityGrassfireConfig& config)
    : config(config)
{
}

void TraversabilityGrassfire::setConfig(const TraversabilityGrassfireConfig& config)
{
    this->config = config;
}

bool TraversabilityGrassfire::calculateTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn, const Eigen::Vector3d& startPos)
{
    // NOTE: CHANGED FROM ENVIRE: Initializations moved here from determineDrivePlane.
    // Make sure temp maps have the correct size.
    grid::Vector2ui numCells = mlsIn.getNumCells();
    bestPatchMap.resize(boost::extents[numCells.y()][numCells.x()]);
    visited.resize(boost::extents[numCells.y()][numCells.x()]);

    // Fill them with default values.
    const SurfacePatchKalman *emptyPatch = NULL;

    // NOTE: Passing NULL directly to std::fill makes the compiler cry....
    std::fill(bestPatchMap.data(), bestPatchMap.data() + bestPatchMap.num_elements(), emptyPatch);
    std::fill(visited.data(), visited.data() + visited.num_elements(), false);

    // Init traversabilityGrid with probability zero and traversability UNKNOWN.
    traversabilityGridOut = grid::TraversabilityGrid(numCells, mlsIn.getResolution(), grid::TraversabilityCell(UNKNOWN, 0));

    // Register classes in traversabilityGrid.
    traversabilityGridOut.setTraversabilityClass(UNKNOWN, grid::TraversabilityClass(1.0));
    traversabilityGridOut.setTraversabilityClass(OBSTACLE, grid::TraversabilityClass(0));
    int numClasses = config.numTraversabilityClasses;
    for(int i = 1; i <= config.numTraversabilityClasses; ++i)
    {
        traversabilityGridOut.setTraversabilityClass(OBSTACLE + i, grid::TraversabilityClass(1.0 / numClasses * i));
    }

    if(!determineDrivePlane(mlsIn, startPos))
    {
        LOG_INFO_S << "TraversabilityGrassfire::Warning, could not find plane robot is driving on";
        return false;
    }

    while(!searchList.empty())
    {
        SearchItem next = searchList.front();
        searchList.pop();

        checkRecursive(mlsIn, next.x, next.y, next.origin);
    }

    computeTraversability(traversabilityGridOut, mlsIn);

    return true;
}

bool TraversabilityGrassfire::determineDrivePlane(const grid::MLSMapKalman& mlsIn, const base::Vector3d& startPos, bool searchSurrounding)
{
    grid::Index startIdx;

    if(!mlsIn.toGrid(startPos, startIdx))
        return false;

    double bestHeightDiff = std::numeric_limits< double >::max();
    const SurfacePatchKalman* bestMatchingPatch = NULL;

    size_t correctedStartX = startIdx.x();
    size_t correctedStartY = startIdx.y();

    // Search the surrounding of the start pos for a start patch.
    for(int i = 0; i < 10; i++)
    {
        for(int yi = -i; yi <= i; yi++)
        {
            for(int xi = -i; xi <= i; xi++)
            {
                // Only check the 'outer rim'.
                if(abs(yi) != i && abs(xi) != i)
                    continue;

                size_t newX = startIdx.x() + xi;
                size_t newY = startIdx.y() + yi;

                grid::Vector2ui numCells = mlsIn.getNumCells();

                if(newX < numCells.x() && newY < numCells.y())
                {
                    bool isObstacle;
                    // Look for the patch with the best height.
                    const SurfacePatchKalman* curPatch = getNearestPatchWhereRobotFits(mlsIn, newX, newY, startPos.z(), isObstacle);
                    if(curPatch)
                    {
                        double curHeightDiff = fabs(startPos.z() - curPatch->getMean() + curPatch->getStandardDeviation());
                        if(curHeightDiff < bestHeightDiff)
                        {
                            bestMatchingPatch = curPatch;
                            bestHeightDiff = curHeightDiff;
                            correctedStartX = newX;
                            correctedStartY = newY;
                        }
                    }
                }
            }
        }
        if(bestMatchingPatch)
            break;
    }

    if(!bestMatchingPatch)
    {
        // We are screwed, can't start the grassfire.
        return false;
    }

    // Recurse to surrounding patches.
    addNeighboursToSearchList(mlsIn, correctedStartX, correctedStartY, bestMatchingPatch);

    return true;
}

const TraversabilityGrassfire::SurfacePatchKalman* TraversabilityGrassfire::getNearestPatchWhereRobotFits(const grid::MLSMapKalman& mlsIn, std::size_t x, std::size_t y, double height, bool& isObstacle)
{
    const SurfacePatchKalman* bestMatchingPatch = NULL;
    double minDistance = std::numeric_limits<double>::max();

    isObstacle = true;

    grid::MLSMapKalman::CellType::const_iterator it = mlsIn.at(x, y).begin();
    grid::MLSMapKalman::CellType::const_iterator itEnd = mlsIn.at(x, y).end();
    for(; it != itEnd; it++)
    {
        // HACK: Filter outliers.
        /* NOTE: CHANGED FROM ENVIRE: Used to use both stddev and a min measurement count.
                 Since "Kalmanpatches" do not store their measurement count anymore,
                 this is no longer possible. */
        if(it->getStandardDeviation() > config.outlierFilterMaxStdDev)
        {
            continue;
        }

        double curFloorHeight = it->getMean() + it->getStandardDeviation();

        // We need to check for patches blocking our way from the current height to this coorindate.
        double curPatchTop = it->getMean() + it->getStandardDeviation();

        // Check if patch is vertical and determine bottom based on that.
        double curPatchBottom;
        if (it->isHorizontal())
            curPatchBottom = it->getMean() - it->getStandardDeviation();
        else
            curPatchBottom = it->getMean() - it->getHeight();

        if(curPatchBottom > (height + config.robotHeight + config.maxStepHeight))
        {
            // The Robot can never reach this patch, so it is not part of the drive plane.
            // Discard it.
            continue;
        }

        // Check if patch blocks the robot 'at the top'.
        if(curPatchBottom < height + config.robotHeight && curPatchTop > height + config.robotHeight)
        {
            // Found a patch that is blocking the robot.
            bestMatchingPatch = &(*it);
            return bestMatchingPatch;
        }

        // Check if patch blocks the robot 'at the ground'.
        if(curPatchBottom < height + config.maxStepHeight && curPatchTop > height + config.maxStepHeight)
        {
            // Found a patch that is blocking the robot.
            bestMatchingPatch = &(*it);
            return bestMatchingPatch;
        }

        double curDist = fabs(curFloorHeight - height);
        if(curDist < minDistance)
        {
            minDistance = curDist;
            bestMatchingPatch =&(*it);
        }
    }

    if(!bestMatchingPatch)
        return bestMatchingPatch;

    double curFloorHeight;
    bool gapTooSmall = true;

    while(gapTooSmall)
    {
        // Now we need to check if there is a blocking patch above this matching patch.
        grid::MLSMapKalman::CellType::const_iterator hcIt= mlsIn.at(x, y).begin();
        grid::MLSMapKalman::CellType::const_iterator hcItEnd = mlsIn.at(x, y).end();

        gapTooSmall = false;
        curFloorHeight = bestMatchingPatch->getMean() + bestMatchingPatch->getStandardDeviation();

        // Check if the selected patch is already an obstacle.
        if(curFloorHeight - height > config.maxStepHeight)
            return bestMatchingPatch;

        for(; hcIt != hcItEnd; hcIt++)
        {
            /**
             * NOTE: CHANGED FROM ENVIRE: Used to use both stddev and a min measurement count.
             *       Since "Kalmanpatches" do not store their measurement count anymore,
             *       this is no longer possible.
             */
            // HACK: Filter outliers.
            if(hcIt->getStandardDeviation() > config.outlierFilterMaxStdDev)
            {
                continue;
            }

            // Check if the robot can pass between this and the other patches.
            double otherCeilingHeight = hcIt->getMean() - hcIt->getStandardDeviation();
            if((curFloorHeight < otherCeilingHeight) && otherCeilingHeight - curFloorHeight < config.robotHeight)
            {
                bestMatchingPatch = &(*hcIt);
                gapTooSmall = true;
                break;
            }
        }

    }
    isObstacle = false;
    return bestMatchingPatch;
}

void TraversabilityGrassfire::addNeighboursToSearchList(const grid::MLSMapKalman& mlsIn, std::size_t x, std::size_t y, const TraversabilityGrassfire::SurfacePatchKalman* patch)
{
    bestPatchMap[y][x] = patch;
    visited[y][x] = true;

    for(int yi = -1; yi <= 1; ++yi)
    {
        for(int xi = -1; xi <= 1; ++xi)
        {
            if(yi == 0 && xi == 0)
                continue;

            size_t newX = x + xi;
            size_t newY = y + yi;

            grid::Vector2ui numCells = mlsIn.getNumCells();

            if(newX < numCells.x() && newY < numCells.y())
            {
                searchList.push(SearchItem(newX, newY, patch));
            }
        }
    }
}

void TraversabilityGrassfire::checkRecursive(const grid::MLSMapKalman& mlsIn, size_t x, size_t y, const SurfacePatchKalman* origin)
{
    if(visited[y][x])
    {
        return;
    }

    visited[y][x] = true;

    bool isKnownObstacle;

    const SurfacePatchKalman* bestMatchingPatch = getNearestPatchWhereRobotFits(mlsIn, x, y, origin->getMean() + origin->getStandardDeviation(), isKnownObstacle);

    if(bestMatchingPatch)
    {
        if(isKnownObstacle)
        {
            bestPatchMap[y][x] = bestMatchingPatch;
            visited[y][x] = true;
        }
        else
        {
            addNeighboursToSearchList(mlsIn, x, y, bestMatchingPatch);
        }
    }
    else
    {
        bestPatchMap[y][x] = NULL;
    }

}

void TraversabilityGrassfire::computeTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn) const
{
    grid::Vector2ui numCells = mlsIn.getNumCells();

    for(size_t y = 0;y < numCells.y(); y++)
    {
        for(size_t x = 0;x < numCells.x(); x++)
        {
            setProbability(traversabilityGridOut, x, y);
            setTraversability(traversabilityGridOut, mlsIn, x, y);
        }
    }
}

void TraversabilityGrassfire::setProbability(grid::TraversabilityGrid& traversabilityGridOut, std::size_t x, std::size_t y) const
{
    const SurfacePatchKalman* currentPatch = bestPatchMap[y][x];
    if(!currentPatch)
    {
        traversabilityGridOut.setProbability(0.0, x, y);
        traversabilityGridOut.setTraversability(UNKNOWN, x, y);
        return;
    }

    /**
     * NOTE: CHANGED FROM ENVIRE: Used to use a nominal measurement count to determine
     *       probability. Since "Kalmanpatches" do not store their measurement count
     *       anymore,this is no longer possible.
     */
    float stdDev = currentPatch->getStandardDeviation();

    if(stdDev < config.nominalStdDev)
    {
        traversabilityGridOut.setProbability(1.0, x, y);
    }
    else
    {
        float probability = config.nominalStdDev / stdDev;
        traversabilityGridOut.setProbability(probability, x, y);
    }
}

void TraversabilityGrassfire::setTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn, size_t x, size_t y) const
{
    const SurfacePatchKalman* currentPatch = bestPatchMap[y][x];
    if(!currentPatch)
    {
        traversabilityGridOut.setTraversability(UNKNOWN, x, y);
        return;
    }

    numeric::PlaneFitting<double> fitter;
    int count = 0;

    double thisHeight = currentPatch->getMean() + currentPatch->getStandardDeviation();

    grid::Vector2d resolution = mlsIn.getResolution();
    const double scaleX = resolution.x();
    const double scaleY = resolution.y();

    LOG_DEBUG_S << "x " << x << " y " << y << " height " << thisHeight << " mean " << currentPatch->getMean() << " stdev " << currentPatch->getStandardDeviation();

    for(int yi = -1; yi <= 1; yi++)
    {
        for(int xi = -1; xi <= 1; xi++)
        {
            if(yi == 0 && xi == 0)
                continue;

            size_t newX = x + xi;
            size_t newY = y + yi;

            grid::Vector2ui numCells = mlsIn.getNumCells();

            if(newX < numCells.x() && newY < numCells.y())
            {
                const SurfacePatchKalman* neighbourPatch = bestPatchMap[newY][newX];
                if(neighbourPatch)
                {
                    count++;
                    double neighbourHeight = neighbourPatch->getMean() + neighbourPatch->getStandardDeviation();

                    LOG_DEBUG_S << "Nx " << newX << " Ny " << newY << " height " << neighbourHeight << " mean " << neighbourPatch->getMean() << " stdev " << neighbourPatch->getStandardDeviation();

                    if(fabs(neighbourHeight - thisHeight) > config.maxStepHeight)
                    {
                        LOG_DEBUG_S << "Step too high.";

                        traversabilityGridOut.setTraversability(OBSTACLE, x, y);
                        return;
                    }

                    Eigen::Vector3d input(xi * scaleX, yi * scaleY, thisHeight - neighbourHeight);

                    LOG_DEBUG_S << "Input to plane fitter: " << input.transpose();

                    fitter.update(input);
                }
                else
                {
                    LOG_DEBUG_S << "Nx " << newX << " Ny " << newY << " is unknown.";
                }
            }
        }
    }

    fitter.update(Eigen::Vector3d(0,0,0));

    if (count < 5)
    {
        LOG_DEBUG_S << "Count too small: " << count << " Setting patch to unknown.";

        traversabilityGridOut.setTraversabilityAndProbability(UNKNOWN, 0.0f, x, y);

        return;
    }

    Eigen::Vector3d fit(fitter.getCoeffs());
    const double divider = std::sqrt(fit.x() * fit.x() + fit.y() * fit.y() + 1);
    double slope = std::acos(1 / divider);

    LOG_DEBUG_S << "Slope is " << slope;

    if(slope > config.maxSlope)
    {
        LOG_DEBUG_S << "Slope is too high.";

        traversabilityGridOut.setTraversability(OBSTACLE, x, y);
        return;
    }

    double drivability = 1.0 - (slope / config.maxSlope);

    //-0.00001 to get rid of precision problems...
    traversabilityGridOut.setTraversability(OBSTACLE + ceil((drivability - 0.00001) * config.numTraversabilityClasses), x, y);

    LOG_DEBUG_S << "Setting traversability class: " << ceil((drivability - 0.00001) * config.numTraversabilityClasses);
}

double TraversabilityGrassfire::getStepHeight(const SurfacePatchKalman* from, const SurfacePatchKalman* to)
{
    return fabs((from->getMean() + from->getStandardDeviation()) - (to->getMean() + to->getStandardDeviation()));
}
