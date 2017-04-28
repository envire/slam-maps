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

/** Based local map **/
#include <maps/grid/MLSMap.hpp>


namespace maps {
namespace grid {






MLSMapSloped generateWaves()
{
    //    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);
    Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //mls_config.updateModel = MLSConfig::KALMAN;
    MLSMapSloped mls = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;


    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();
    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double sn = std::sin(y * M_PI/2.5);
            mls.mergePoint(Eigen::Vector3d(x, y, cs*sn));
        }
    }
    return mls;
}

}  // namespace grid
}  // namespace maps
