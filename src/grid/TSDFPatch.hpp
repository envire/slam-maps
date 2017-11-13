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
#ifndef __MAPS_TSDF_PATCH_HPP_
#define __MAPS_TSDF_PATCH_HPP_


#include <base/Float.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>


namespace maps { namespace grid
{

class TSDFPatch
{
    float distance;
    float var;

    template <class T> static inline void kalman_update( T& mean, T& var, T m_mean, T m_var )
    {
	float gain = var / (var + m_var);
	if( gain != gain )
	    gain = 0.5f; // this happens when both vars are 0.
	mean = mean + gain * (m_mean - mean);
	var = (1.0f - gain) * var;
    }

public:
    TSDFPatch() : distance(base::NaN<float>()), var(1.f) {}
    TSDFPatch(float distance, float var) : distance(distance), var(var) {}
    virtual ~TSDFPatch() {}

    void update(float distance, float var, float truncation = 1.f, float min_var = 0.001f)
    {
	if(base::isNaN<float>(this->distance))
	    this->distance = distance;

	kalman_update(this->distance, this->var, distance, var);

	if(this->var < min_var)
	    this->var = min_var;

	if(distance > truncation)
	    distance = truncation;
	else if(distance < -truncation)
	    distance = -truncation;
    }

    float getDistance() const
    {
	return distance;
    }

    float getVariance() const
    {
	return var;
    }

    float getStandardDeviation() const
    {
	return std::sqrt(var);
    }

    bool operator==(const TSDFPatch& other) const
    {
	return this == &other;
    }

protected:

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
	ar & BOOST_SERIALIZATION_NVP(distance);
	ar & BOOST_SERIALIZATION_NVP(var);
    }
};

}  //namespace grid
}  //namespace maps

#endif  //__MAPS_TSDF_PATCH_HPP_