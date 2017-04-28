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
#pragma once

#include "SurfacePatches.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

namespace maps { namespace grid
{

struct OccupancyConfiguration
{
    OccupancyConfiguration(double hit_probability = 0.7, double miss_probability = 0.4,
                         double occupied_probability = 0.8, double free_space_probability = 0.3,
                         double max_probability = 0.971, double min_probability = 0.1192,
                         float uncertainty_threshold = 0.25) :
                        hit_logodds(OccupancyPatch::logodds(hit_probability)),
                        miss_logodds(OccupancyPatch::logodds(miss_probability)),
                        occupied_logodds(OccupancyPatch::logodds(occupied_probability)),
                        free_space_logodds(OccupancyPatch::logodds(free_space_probability)),
                        max_logodds(OccupancyPatch::logodds(max_probability)) ,
                        min_logodds(OccupancyPatch::logodds(min_probability)),
                        uncertainty_threshold(uncertainty_threshold) {}

    float hit_logodds;
    float miss_logodds;
    float occupied_logodds;
    float free_space_logodds;
    float max_logodds;
    float min_logodds;
    float uncertainty_threshold;

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(hit_logodds);
        ar & BOOST_SERIALIZATION_NVP(miss_logodds);
        ar & BOOST_SERIALIZATION_NVP(occupied_logodds);
        ar & BOOST_SERIALIZATION_NVP(free_space_logodds);
        ar & BOOST_SERIALIZATION_NVP(max_logodds);
        ar & BOOST_SERIALIZATION_NVP(min_logodds);
        ar & BOOST_SERIALIZATION_NVP(uncertainty_threshold);
    }
};

}}
