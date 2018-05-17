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
#ifndef __MAPS_SIMPLE_TRAVERSABILITY_HPP_
#define __MAPS_SIMPLE_TRAVERSABILITY_HPP_

#include <maps/grid/GridMap.hpp>
#include <maps/grid/TraversabilityGrid.hpp>

namespace maps { namespace tools
{
    /** @brief Configuration parameters for the SimpleTraversability operator
     *
     * See SimpleTraversability for more information on the use of these
     * parameters.
     */
    struct SimpleTraversabilityConfig
    {
        /** The maximum slope the robot is able to traverse */
        double maximumSlope;

        /** Number of classes in the output map */
        int classCount;

        /**
        * Required ground clearance, in meters. A cell is classified as an
        * obstacle if the maximum step around that cell is higher than the
        * ground clearance
        */
        double groundClearance;

        /**
        * Traversable passages that are narrower (in m) than this will be
        * closed
        */
        double minPassageWidth;

        /**
        * Clearance (in m) around obstacles. When set to a value larger than
        * 0, all obstacles will be grown by a border of given width.
        */
        double obstacleClearance;

        SimpleTraversabilityConfig()
            : maximumSlope(0)
            , classCount(0)
            , groundClearance(0)
            , minPassageWidth(0)
            , obstacleClearance(0)
        {
        }

        SimpleTraversabilityConfig(
                double maximumSlope,
                int classCount,
                double groundClearance,
                double minPassageWidth = 0,
                double obstacleClearance = 0)

            : maximumSlope(maximumSlope)
            , classCount(classCount)
            , groundClearance(groundClearance)
            , minPassageWidth(minPassageWidth)
            , obstacleClearance(obstacleClearance)
        {
        }

    };

    /**
     * @brief Classification of terrain into symbolic traversability classes, based on
     * geometric constraints
     *
     * For now, the modalities that are used are:
     *
     * <ul>
     * <li>maximum slope
     * <li>maximum step size
     * </ul>
     *
     * The (very simple) model maps the local slope from [@a maximumSlope, 1] to [0, 1].
     * The resulting value is then quantified in @a classCount intervals.
     * Probability will be set to 1 for every classification beside CLASS_UNKNOWN.
     *
     * It outputs a TraversabilityGrid in which each cell has an integer value,
     * this integer value being the traversability class for the cell. Since
     * CLASS_UNKNOWN (=0) and CLASS_OBSTACLE (=1) are reserved, the resulting
     * traversability class is in [2, 2 + classCount - 1]
     *
     * If one of the modalities is missing, it is simply ignored
     */
    class SimpleTraversability {

    private:

        SimpleTraversabilityConfig config;

    public:

        enum CLASSES {
            CLASS_UNKNOWN = 0,
            CLASS_OBSTACLE = 1,
            CUSTOM_CLASSES = 2
        };

        SimpleTraversability();

        SimpleTraversability(
                const SimpleTraversabilityConfig& config);

        SimpleTraversability(
                double maximumSlope,
                unsigned int classCount,
                double groundClearance,
                double minPassageWidth = 0,
                double obstacleClearance = 0);

        void setConfig(const SimpleTraversabilityConfig& config);
        void setConfig(
                double maximumSlope,
                unsigned int classCount,
                double groundClearance,
                double minPassageWidth = 0,
                double obstacleClearance = 0);

        const SimpleTraversabilityConfig& getConfig() const;

        /**
         * Calculates the traversability based on the input maps.
         * @details:
         * Note that the default value of the input maps will be
         * used to determine UNKNOWN cells.
         */
        bool calculateTraversability(grid::TraversabilityGrid& traversabilityOut,
                                     const grid::GridMapF& slopesIn,
                                     const grid::GridMapF& maxStepsIn) const;

        /**
         * Closes spaces between obstacles <= @a minPassageWidth.
         * Will be called by @ calculateTraversability
         * (before growObstacles) if
         * minPassageWidth is greater than 0.
         */
        void closeNarrowPassages(maps::grid::TraversabilityGrid& traversabilityGrid, double minPassageWidth) const;

        /**
         * Grows obstacles by growthRadius.
         * Will be called by calculateTraversability
         * (after closeNarrowPassages) if
         * obstacleClearance is greater than 0.
         */
        void growObstacles(maps::grid::TraversabilityGrid& traversabilityGrid, double growthRadius) const;

    };

}  // End namespace tools
}  // End namespace maps

#endif  //__MAPS_SIMPLE_TRAVERSABILITY_HPP_
