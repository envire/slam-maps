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