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
#include <Eigen/Core>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>


namespace maps { namespace grid
{

class TSDFPatch
{
    float distance;
    float var;
    /** mean color (rgb in [0,1]) of the samples added with addColor(), and their summed weight */
    Eigen::Vector3f color;
    float color_weight;

    template <class T> static inline void kalman_update( T& mean, T& var, T m_mean, T m_var )
    {
	float gain = var / (var + m_var);
	if( gain != gain )
	    gain = 0.5f; // this happens when both vars are 0.
	mean = mean + gain * (m_mean - mean);
	var = (1.0f - gain) * var;
    }

public:
    TSDFPatch() : distance(base::NaN<float>()), var(1.f), color(Eigen::Vector3f::Zero()), color_weight(0.f) {}
    TSDFPatch(float distance, float var) : distance(distance), var(var), color(Eigen::Vector3f::Zero()), color_weight(0.f) {}
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

    /** adds a color sample (rgb in [0,1]) to the running mean of the patch color */
    void addColor(const Eigen::Vector3f& rgb, float weight = 1.f)
    {
	if(weight <= 0.f)
	    return;
	color_weight += weight;
	color += (weight / color_weight) * (rgb - color);
    }

    bool hasColor() const
    {
	return color_weight > 0.f;
    }

    /** mean color of the samples added, black without any (see hasColor()) */
    const Eigen::Vector3f& getColor() const
    {
	return color;
    }

    float getColorWeight() const
    {
	return color_weight;
    }

    void clearColor()
    {
	color.setZero();
	color_weight = 0.f;
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
	if(version >= 1)
	{
	    ar & boost::serialization::make_nvp("color_r", color.x());
	    ar & boost::serialization::make_nvp("color_g", color.y());
	    ar & boost::serialization::make_nvp("color_b", color.z());
	    ar & BOOST_SERIALIZATION_NVP(color_weight);
	}
	else
	{
	    color.setZero();
	    color_weight = 0.f;
	}
    }
};

}  //namespace grid
}  //namespace maps

BOOST_CLASS_VERSION(maps::grid::TSDFPatch, 1);

#endif  //__MAPS_TSDF_PATCH_HPP_