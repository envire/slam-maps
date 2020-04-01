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
#ifndef __MAPS_TRAVERSABILITY_GRASSFIRE_CONFIG_HPP_
#define __MAPS_TRAVERSABILITY_GRASSFIRE_CONFIG_HPP_

namespace maps { namespace tools
{
    class TraversabilityGrassfireConfig
    {
    public:
        TraversabilityGrassfireConfig()
            : maxStepHeight(0)
            , maxSlope(0)
            , robotHeight(0)
            , numTraversabilityClasses(0)
            , nominalStdDev(1)
            , outlierFilterMaxStdDev(0.0)
        {
        };

        TraversabilityGrassfireConfig(double maxStepHeight,
                                      double maxSlope,
                                      double robotHeight,
                                      int numTraversabilityClasses,
                                      double nominalStdDev,
                                      double outlierFilterMaxStdDev)
            : maxStepHeight(maxStepHeight)
            , maxSlope(maxSlope)
            , robotHeight(robotHeight)
            , numTraversabilityClasses(numTraversabilityClasses)
            , nominalStdDev(nominalStdDev)
            , outlierFilterMaxStdDev(outlierFilterMaxStdDev)
            {
            };

        double maxStepHeight;
        double maxSlope;
        double robotHeight;
        int numTraversabilityClasses;

        /**
         * NOTE: CHANGED FROM ENVIRE: Used to use both stddev and a min measurement count.
         *       Since "Kalmanpatches" do not store their measurement count anymore,
         *       this is no longer possible.
         *
         * The standard deviation a MLS-Patch needs
         * to get a probability of 1.0 (lower == better).
         */
        double nominalStdDev;

        double outlierFilterMaxStdDev;
    };
}  // End namespace tools.
}  // End namespace maps.

#endif  // __MAPS_TRAVERSABILITY_GRASSFIRE_CONFIG_HPP_
