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
#ifndef __MAPS_SIMPLE_TRAVERSABILITY_RADIAL_LUT_HPP_
#define __MAPS_SIMPLE_TRAVERSABILITY_RADIAL_LUT_HPP_

#include "maps/grid/TraversabilityGrid.hpp"
#include "boost/multi_array.hpp"

namespace maps { namespace tools
{

    /**
     * Radial lookup table used in SimpleTraversability::closeNarrowPassages.
    **/
    class SimpleTraversabilityRadialLUT
    {
    private:
        int centerX, centerY;
        unsigned int width, height;
        boost::multi_array<std::pair<int, int>, 2>  parents;
        boost::multi_array<bool, 2> inDistance;

    public:
        /**
        * Sets up parents and inDistance so they cover a grid of
        * (@a distance * 2) x (@a distance * 2) with an additional
        * Cell as the center.
        */
        void precompute(double distance, double resolutionX, double resolutionY);

        /**
        * Checks for the targetedValue within the in distance around the given center and calls markSingleRadius on them.
        */
        void markAllRadius(grid::TraversabilityGrid& traversabilityGrid, int centerX, int centerY, int targetedValue) const;

        /**
         * Marks all cells between target and center with the @a markValue using the @a parents table.
         */
        void markSingleRadius(grid::TraversabilityGrid& traversabilityGrid, int centerX, int centerY, int targetX, int targetY, int expectedValue, int markValue) const;

    };

}  // End namespace tools.
}  // End namespace maps.

#endif  // __MAPS_SIMPLE_TRAVERSABILITY_RADIAL_LUT_HPP_
