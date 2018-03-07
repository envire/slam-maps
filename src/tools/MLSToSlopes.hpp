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
#ifndef __MAPS_MLS_TO_SLOPES_HPP_
#define __MAPS_MLS_TO_SLOPES_HPP_

#include "../grid/GridMap.hpp"
#include "../grid/MLSMap.hpp"

namespace maps { namespace tools 
{
    /** This operator computes local slopes on a MLS map
     *
     * It acts on an MLSMapKalman and updates a GridMap<float> with the maximum local
     * slope angles in radians.
     * It also returns the absolute value of the maximal step between each cell and its direct
     * neighbours.
     *
     * The default operation will compute the maximum slope between the topmost
     * surfaces of a MLS. In practice, it means that it works only on MLS that
     * have one patch per cell.
     */
    class MLSToSlopes
    {

    public:
        
        /**
        * @brief: Compute maxSteps from MLSMapKalman.
        * @details: The output GridMap will be fitted to match the size and resolution of the input.
        * If not enough information is available for a cell all ouput values are set to -inf.
        * @param useStdDev : Set to true to include standard deviation in the calculation of max steps.
        * @param correctSteps :  Set to true to compute corrected max steps instead.
        * @param correctedStepThreshold : If the value for a max step falls under the threshold, it will be corrected.
        * */
        static bool computeMaxSteps(const maps::grid::MLSMapKalman& mlsIn, maps::grid::GridMapF& maxStepsOut,
                                    bool useStdDev = false, bool correctSteps = false, float correctedStepThreshold = 0);
        
        /**
         * @brief Compute slopes from MLSMapKalman.
         * @param windowSize: The slope for each cell is computed from -windowSize to windowSize around the cell. Has to be >= 1.
         **/
        static bool computeSlopes(const maps::grid::MLSMapKalman& mlsIn, maps::grid::GridMapF& slopesOut, int windowSize = 1);
        
    };
    
}  // end namespace tools
}  // end namespace maps
  
#endif  // end __MAPS_MLS_TO_SLOPES_HPP_















