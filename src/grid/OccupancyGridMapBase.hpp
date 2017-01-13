#pragma once

#include "OccupancyConfiguration.hpp"

#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/assume_abstract.hpp>

namespace maps { namespace grid
{

class OccupancyGridMapBase
{
public:
    OccupancyGridMapBase(const OccupancyConfiguration& config) : config(config) {}
    virtual ~OccupancyGridMapBase() {}

    virtual void mergePoint(const Eigen::Vector3d& sensor_origin, const Eigen::Vector3d& measurement) = 0;
    virtual bool isOccupied(const Eigen::Vector3d& point) const = 0;
    virtual bool isOccupied(Index idx, float z) const = 0;
    virtual bool isFreeSpace(const Eigen::Vector3d& point) const = 0;
    virtual bool isFreeSpace(Index idx, float z) const = 0;
    virtual bool hasSameFrame(const base::Transform3d& local_frame, const Vector2ui &num_cells, const Vector2d &resolution) const = 0;

    const OccupancyConfiguration& getConfig() const {return config;}

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(config);
    }

    OccupancyConfiguration config;
};

}}

BOOST_SERIALIZATION_ASSUME_ABSTRACT(maps::grid::OccupancyGridMapBase);