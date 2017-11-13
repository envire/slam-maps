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
#ifndef __MAPS_OCCUPANCY_PATCH_HPP_
#define __MAPS_OCCUPANCY_PATCH_HPP_


#include <cmath>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>


namespace maps { namespace grid 
{   

class OccupancyPatch
{
    float log_odds;

public:
    OccupancyPatch(double initial_probability) : log_odds(logodds(initial_probability)) {}
    OccupancyPatch(float initial_log_odds = 0.f) : log_odds(initial_log_odds) {}
    virtual ~OccupancyPatch() {}

    double getPropability() const
    {
	return probability(log_odds);
    }

    float getLogOdds() const
    {
	return log_odds;
    }

    bool isOccupied(double occupied_tresshold = 0.8) const
    {
	return probability(log_odds) >= occupied_tresshold;
    }

    bool isFreeSpace(double not_occupied_tresshold = 0.3) const
    {
	return probability(log_odds) < not_occupied_tresshold;
    }

    void updatePropability(double update_prob, double min_prob = 0.1192, double max_prob = 0.971)
    {
	updateLogOdds(logodds(update_prob), logodds(min_prob), logodds(max_prob));
    }

    void updateLogOdds(float update_logodds, float min = -2.f, float max = 3.5f)
    {
	log_odds += update_logodds;
	if(log_odds < min)
	    log_odds = min;
	else if(log_odds > max)
	    log_odds = max;
    }

    bool operator==(const OccupancyPatch& other) const
    {
	return this == &other;
    }

    // compute log-odds from probability
    static inline float logodds(double probability)
    {
	return (float)log(probability / (1.0 - probability));
    }

    // compute probability from log-odds
    static inline double probability(double logodds)
    {
	return 1.0 - ( 1.0 / (1.0 + exp(logodds)));
    }

protected:

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
	ar & BOOST_SERIALIZATION_NVP(log_odds);
    }
};
	
}  //namespace grid
}  //namespace maps

#endif  //__MAPS_OCCUPANCY_PATCH_HPP_