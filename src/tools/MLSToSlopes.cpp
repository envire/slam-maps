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
#include "MLSToSlopes.hpp"
#include <boost/multi_array.hpp>
#include <numeric/PlaneFitting.hpp>
#include <maps/grid/Index.hpp>

using namespace maps;
using namespace tools;
using namespace Eigen;

static double const UNKNOWN = -std::numeric_limits<double>::infinity();

static void updateDiffs(grid::MLSMapKalman const& mlsIn,
        bool useStdDev,
        boost::multi_array<float,3>& diffs,
        boost::multi_array<int, 2>& count,
        int this_index, size_t this_x, size_t this_y,
        int other_index, size_t other_x, size_t other_y,
        grid::MLSMapKalman::CellType::const_iterator this_patch)
{
    const grid::MLSMapKalman::CellType& neighbour_cell = mlsIn.at(grid::Index(other_x, other_y));
    grid::MLSMapKalman::CellType::const_iterator neighbour_patch =
        std::max_element(neighbour_cell.begin(), neighbour_cell.end());
    
    if (neighbour_patch != neighbour_cell.end())
    {
        float z0 = this_patch->getMean();
        float z1 = neighbour_patch->getMean();
        float stdev0 = 0;
        float stdev1 = 0;
        if (useStdDev)
        {
            stdev0 = this_patch->getStandardDeviation();
            stdev1 = neighbour_patch->getStandardDeviation();
        }
        
        if (z0 > z1)
        {
            std::swap(z0, z1);
            std::swap(stdev0, stdev1);
        }
        
        double min_z = z0 - stdev0;
        double max_z = z1 + stdev1;
        
        double step = max_z - min_z;
        
        diffs[this_y][this_x][this_index] = step;
        count[this_y][this_x]++;
        
        diffs[other_y][other_x][other_index] = step;
        count[other_y][other_x]++;
    }
    
    else
    {
        diffs[this_y][this_x][this_index] = UNKNOWN;
        diffs[other_y][other_x][other_index] = UNKNOWN;
    }
}

bool MLSToSlopes::computeMaxSteps(const grid::MLSMapKalman& mlsIn, grid::GridMapF& maxStepsOut,
                                  bool useStdDev, bool correctSteps, float correctedStepThreshold)
{
    // Input has to have width and height > 0.
    size_t width = mlsIn.getNumCells()[0];
    size_t height = mlsIn.getNumCells()[1];
    
    if( width == 0 || height == 0 )
        return false;
    
    // Fit the output to the size and resolution of the input map and initialize with UNKNOWN.
    maxStepsOut = grid::GridMapF(mlsIn.getNumCells(), mlsIn.getResolution(), UNKNOWN);
    
    boost::multi_array<int,2> counts;
    counts.resize(boost::extents[(size_t) mlsIn.getNumCells()[1]][(size_t) mlsIn.getNumCells()[0]]);
    std::fill(counts.data(), counts.data() + counts.num_elements(), 0);
    
    boost::multi_array<float,3> diffs;
    diffs.resize(boost::extents[(size_t) mlsIn.getNumCells()[1]][(size_t) mlsIn.getNumCells()[0]][8]);
    std::fill(diffs.data(), diffs.data() + diffs.num_elements(), 0);
    
    
    static const int
        BOTTOM_CENTER = 0,
        TOP_CENTER = 1,
        TOP_RIGHT = 2,
        BOTTOM_LEFT = 3,
        CENTER_RIGHT = 4,
        CENTER_LEFT = 5,
        BOTTOM_RIGHT = 6,
        TOP_LEFT = 7;
    
    // Calculate diffs between the cells.
    for (size_t y = 1; y < height-1; ++y)
    {
        for (size_t x = 1; x < width; ++x)
        {
            const grid::MLSMapKalman::CellType& this_cell = mlsIn.at(grid::Index(x, y));  
            grid::MLSMapKalman::CellType::const_iterator this_patch = 
                        std::max_element(this_cell.begin(), this_cell.end());
            
            if (this_patch == this_cell.end())
                continue;
            
            // Compute diffs between the current cell and each neighbour.
            updateDiffs(mlsIn, useStdDev,  diffs, counts, 
                    BOTTOM_CENTER, x, y, TOP_CENTER, x, y + 1, this_patch);
            updateDiffs(mlsIn, useStdDev, diffs, counts,
                    TOP_RIGHT, x, y, BOTTOM_LEFT, x - 1, y - 1, this_patch);
            updateDiffs(mlsIn, useStdDev, diffs, counts,
                    CENTER_RIGHT, x, y, CENTER_LEFT, x - 1, y, this_patch);
            updateDiffs(mlsIn, useStdDev, diffs, counts,
                    BOTTOM_RIGHT, x, y, TOP_LEFT, x - 1, y + 1, this_patch);
        }
    }
    
    // Calculate max_steps and corrected_step_threshold from diffs
    for (size_t y = 1; y < (height - 1); ++y)
    {
        for (size_t x = 1; x < (width - 1); ++x)
        {
            int count = counts[y][x];
            if (count < 5)
            {
                continue;
            }
            
            double max_step = UNKNOWN;
            double corrected_max_step = UNKNOWN;
            for (int i = 0; i < 8; i += 2)
            {
                double step0 = diffs[y][x][i];
                double step1 = diffs[y][x][i + 1];
                max_step = std::max(max_step, step0);
                max_step = std::max(max_step, step1);
                corrected_max_step = std::max(corrected_max_step, step0 - (step0 + step1) / 4);
                corrected_max_step = std::max(corrected_max_step, step0 - (step0 + step1) * 3 / 4);
            }
            if (correctSteps && max_step < correctedStepThreshold)
                maxStepsOut.at(grid::Index(x, y)) = corrected_max_step;
            else
                maxStepsOut.at(grid::Index(x, y)) = max_step;
        }
    }

    return true;
}

bool MLSToSlopes::computeSlopes(const grid::MLSMapKalman& mlsIn, grid::GridMapF& slopesOut, int windowSize)
{
    // Input has to have width and height > 0.
    size_t width = mlsIn.getNumCells()[0];
    size_t height = mlsIn.getNumCells()[1];
    
    if( width == 0 || height == 0 )
        return false;
    
    // Fit the output to the size and resolution of the input map and initialize with UNKNOWN.
    slopesOut = grid::GridMapF(mlsIn.getNumCells(), mlsIn.getResolution(), UNKNOWN);
    
    double scalex = mlsIn.getResolution()[0];
    double scaley = mlsIn.getResolution()[1];
    
    for (size_t y = 1; y < height-1; ++y)
    {
        for (size_t x = 1; x < width-1; ++x)
        {
            const grid::MLSMapKalman::CellType& this_cell = mlsIn.at(grid::Index(x, y));  
            grid::MLSMapKalman::CellType::const_iterator this_patch = 
                        std::max_element(this_cell.begin(), this_cell.end());
            if (this_patch == this_cell.end())
            {
                continue;
            }
            
            // Compute gradient in 2 * windowSize area around the current cell.
            numeric::PlaneFitting<double> fitter;
            int count = 0;
            double thisHeight = this_patch->getMean();
            for (int yi = -windowSize; yi <= windowSize; ++yi) {
                for (int xi = -windowSize; xi <= windowSize; ++xi) {
                    //skip own entry
                    if (xi == 0 && yi == 0)
                        continue;
                    
                    const int rx = x + xi;
                    const int ry = y + yi;
                    
                    if ((rx < 0) || (rx >= (int) width) || (ry < 0) || (ry >= (int) height) )
                        continue;
                    
                    const grid::MLSMapKalman::CellType& neighbour_cell = mlsIn.at(grid::Index(rx, ry));
                    grid::MLSMapKalman::CellType::const_iterator neighbour_patch =
                        std::max_element(neighbour_cell.begin(), neighbour_cell.end());
                    
                    if (neighbour_patch != neighbour_cell.end())
                    {
                        count++;
                        Vector3d point(xi * scalex, yi * scaley, thisHeight - neighbour_patch->getMean());
                        fitter.update(point);
                    }
                }
            }
            
            fitter.update(Vector3d(0, 0, 0));
            
            if (count < 5)
            {
                slopesOut.at(grid::Index(x, y)) = UNKNOWN;
            }
            else
            {
                Vector3d fit(fitter.getCoeffs());
                const double divider = sqrt(fit.x() * fit.x() + fit.y() * fit.y() + 1);
                slopesOut.at(grid::Index(x, y)) = acos(1 / divider);
            }
        }
    }
    
    return true;
}
