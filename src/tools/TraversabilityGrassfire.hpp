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
#ifndef __MAPS_TRAVERSABILITY_GRASSFIRE_HPP_
#define __MAPS_TRAVERSABILITY_GRASSFIRE_HPP_

#include <base/Eigen.hpp>

#include <Eigen/Core>

#include <maps/grid/MLSConfig.hpp>
#include <maps/grid/MLSMap.hpp>
#include <maps/grid/SurfacePatches.hpp>
#include <maps/grid/TraversabilityGrid.hpp>

#include <maps/tools/TraversabilityGrassfireConfig.hpp>
#include <maps/tools/TraversabilityGrassFireSearchItem.hpp>

#include <queue>

namespace maps { namespace tools
{
/**
 * @brief: TraversabilityGrassfire is a tool to calvulate the traversability
 *         of an area based on maxSteps and slopes.
 * @details: TraversabilityGrassfire iterates recursively over the given mls
 *           from the @param startPos. Any unreachable areas will not be
 *           included.
 *           Always set a @param config before calling @fn calculateTraversability.
 */
class TraversabilityGrassfire
{
    typedef grid::SurfacePatch<grid::MLSConfig::KALMAN> SurfacePatchKalman;
    typedef TraversabilityGrassfireSearchItem SearchItem;

public:
    TraversabilityGrassfire();
    TraversabilityGrassfire(const TraversabilityGrassfireConfig& config);

    void setConfig(const TraversabilityGrassfireConfig& config);
    bool calculateTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn, const Eigen::Vector3d& startPos);

private:
    bool determineDrivePlane(const grid::MLSMapKalman& mlsIn, const base::Vector3d& startPos, bool searchSurrounding = true);
    const SurfacePatchKalman* getNearestPatchWhereRobotFits(const grid::MLSMapKalman& mlsIn, size_t x, size_t y, double height, bool& isObstacle);
    void addNeighboursToSearchList(const grid::MLSMapKalman& mlsIn, std::size_t x, std::size_t y, const SurfacePatchKalman* patch);
    void checkRecursive(const grid::MLSMapKalman& mlsIn, size_t x, size_t y, const SurfacePatchKalman* origin);

    void computeTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn) const;
    void setProbability(grid::TraversabilityGrid& traversabilityGridOut, size_t x, size_t y) const;
    void setTraversability(grid::TraversabilityGrid& traversabilityGridOut, const grid::MLSMapKalman& mlsIn, size_t x, size_t y) const;

    double getStepHeight(const SurfacePatchKalman* from, const SurfacePatchKalman* to);
    void markAsObstacle(size_t x, size_t y);

    TraversabilityGrassfireConfig config;

    std::queue<SearchItem> searchList;

    boost::multi_array<bool, 2> visited;
    boost::multi_array<const SurfacePatchKalman*, 2> bestPatchMap;

    enum TRCLASSES
    {
        UNKNOWN = 0,
        OBSTACLE = 1,
    };
};

}  // End namespace tools
}  // End namespace maps

#endif // __MAPS_TRAVERSABILITY_GRASSFIRE_HPP_
