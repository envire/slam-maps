#ifndef __MAPS_SURFACEPATCHES_HPP_
#define __MAPS_SURFACEPATCHES_HPP_

#include <numeric/PlaneFitting.hpp>

#include <base/Eigen.hpp>

#include <Eigen/Core>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

#include <boost_serialization/BaseNumeric.hpp>

#include "MLSConfig.hpp"


namespace maps { namespace grid
{
//! global typedef for all SurfacePatches. Only float seems to be useful.
typedef float SPScalar;
typedef Eigen::Matrix<SPScalar, 3, 1> Vector3;
typedef Eigen::Matrix<SPScalar, 3, 3> Matrix33;


class SurfacePatchBase
{
protected:
    float min, max;

public:
    // TODO TYPE is for compatibility with old Patches -- probably should be made optional
    enum TYPE
    {
        VERTICAL = 0,
        HORIZONTAL = 1,
        NEGATIVE = 2
    };

    // TODO set default value properly
    SurfacePatchBase()
        : min(0),
          max(0)
    {}

    explicit SurfacePatchBase(const float &z, const size_t& updateIdx = 0)
    : min(z), max(z)
    {}

    explicit SurfacePatchBase(const float &mean, const float &height, const size_t& updateIdx = 0)
    : min(mean - height), max(mean)
    {}

    bool merge(const SurfacePatchBase& other, const float& gapSize=0.0f)
    {
        if(!isCovered(other, gapSize)) return false;
        min = std::min(min, other.min);
        max = std::max(max, other.max);
        return true;
    }

    float getMin() const { return min; }
    float getMax() const { return max; }
    float getTop() const { return max; }
    float getBottom() const { return min; }
    void getRange(float& mini, float& maxi) const {mini = min; maxi = max; }

    bool isCovered(const float& z, const float& gapSize = 0.0f) const
    {
        return min - gapSize < z && z < max + gapSize;
    }

    bool isCovered(const SurfacePatchBase& other, const float& gapSize) const
    {
        return min - gapSize < other.max && other.min - gapSize < max;
    }

    bool operator<(const SurfacePatchBase& other) const
    {
        return min < other.min;
    }

    bool operator==(const SurfacePatchBase& other) const
    {
        return min == other.min && max == other.max;
    }

    bool isNegative() const
    {
        return false; // base patch does not allow negative patches
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(min);
        ar & BOOST_SERIALIZATION_NVP(max);
    }
};

/**
 * Templated surface patch class.
 * TODO add color option
 */
template<MLSConfig::update_model model>
class SurfacePatch;


/**
 * SurfacePatch type for SLOPE update model.
 */
template<>
class SurfacePatch<MLSConfig::SLOPE> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    numeric::PlaneFitting<float> plane;
    float n;
    // FIXME type is useless at the moment (but removing it will break compatibility with serialized maps)
    TYPE type;
public:
    SurfacePatch() : n(0), type(TYPE::HORIZONTAL)
    {}

    SurfacePatch(const Eigen::Vector3f& point, const float& cov)
        : Base(point.z())
        , plane(point, 1.0f/cov)
        , n(1), type(TYPE::HORIZONTAL)
    {}

    SurfacePatch(const float& mean, const float& stdev, const float& height = 0, TYPE type_=TYPE::HORIZONTAL)
        : Base(mean, height)
        , n(1), type(type_)
    {}


    /**
     * Compares two patches by their center of mass
     */
    bool operator<(const SurfacePatch& other) const
    {
        return plane.z * other.plane.n < other.plane.z * plane.n;
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator ==(other) &&
                plane.n == other.plane.n && plane.x == other.plane.x &&
                plane.y == other.plane.y && plane.z == other.plane.z &&
                plane.xx == other.plane.xx && plane.xy == other.plane.xy &&
                plane.xz == other.plane.xz && plane.yy == other.plane.yy &&
                plane.yz == other.plane.yz && plane.zz == other.plane.zz &&
                n == other.n && type == other.type;
    }

    /**
     * returns the top most
     */
    float getTop() const
    {
        // TODO
        return Base::getMax();
    }

    float getBottom() const
    {
        // TODO
        return Base::getMin();
    }

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(plane.x, plane.y, plane.z);
        return center * (1.0f/plane.n);
    }

    Eigen::Vector3f getNormal() const
    {
        if(n<=1.0f) return Eigen::Vector3f::UnitZ();
        return plane.getNormal();
    }

    bool merge(const SurfacePatch& other, const MLSConfig& config)
    {
        if(Base::merge(other, config.gapSize))
        {
            plane.update(other.plane);
            n+= other.n;
            return true;
        }

        return false;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar & BOOST_SERIALIZATION_NVP(plane);
        ar & BOOST_SERIALIZATION_NVP(n);
        ar & BOOST_SERIALIZATION_NVP(type);
    }    
}; // SurfacePatch<MLSConfig::SLOPE>

template<>
class SurfacePatch<MLSConfig::KALMAN>: public SurfacePatchBase
{
public:
    typedef SurfacePatchBase Base;
    float mean, var, height;

    template <class T> static inline void kalman_update( T& mean, T& var, T m_mean, T m_var )
    {
        float gain = var / (var + m_var);
        if( gain != gain )
            gain = 0.5f; // this happens when both vars are 0.
        mean = mean + gain * (m_mean - mean);
        var = (1.0f-gain)*var;
    }


public:
    SurfacePatch() : mean(0), var(0), height(0)
    {}

    SurfacePatch(const Eigen::Vector3f& point, const float& cov)
        : Base(point.z())
        , mean(point.z()), var(cov), height(0)
    {}

    SurfacePatch(const float& mean, const float& stdev, const float& height = 0, TYPE type_=TYPE::HORIZONTAL)
        : Base(mean, height)
        , mean(mean), var(stdev*stdev), height(height)
    {}

    bool merge(const SurfacePatch& other, const MLSConfig& config)
    {
        if( !Base::merge(other, config.gapSize))
            return false;


        float delta_dev = std::sqrt(var + other.var);
        if(height==0.0f && other.height == 0.0f
                && (mean - config.thickness - delta_dev) < other.mean &&
                (mean + config.thickness + delta_dev) > other.mean )
        {
            kalman_update(mean, var, other.mean, other.var);
        }
        else
        {
            height = std::max(mean, other.mean) - std::min(mean - height, other.mean - other.height);
            if(mean < other.mean)
            {
                mean = other.mean;
                var = other.var;
            }
        }

        return true;
    }

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(0.0f, 0.0f, mean);
        return center;
    }

    bool operator<(const SurfacePatch& other) const
    {
        return mean < other.mean;
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator ==(other) && mean == other.mean && var == other.var && height == other.height;
    }

    Eigen::Vector3f getNormal() const
    {
        return Eigen::Vector3f::UnitZ();
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar & BOOST_SERIALIZATION_NVP(mean);
        ar & BOOST_SERIALIZATION_NVP(var);
        ar & BOOST_SERIALIZATION_NVP(height);
    }        

}; // SurfacePatch<MLSConfig::KALMAN>


/**
 * SurfacePatch type for data exchange
 */
template<>
class SurfacePatch<MLSConfig::PRECALCULATED> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    Eigen::Vector3f center, normal;
public:
    template<MLSConfig::update_model model>
    SurfacePatch(const SurfacePatch<model>& other) : SurfacePatchBase(other), center(other.getCenter()), normal(other.getNormal())
    {
        // empty
    }
    template<MLSConfig::update_model model>
    SurfacePatch& operator=(const SurfacePatch<model>& other)
    {
        new(this) SurfacePatch(other);
        return *this;
    }

    const Eigen::Vector3f& getCenter() const { return center; }
    Eigen::Vector3f& getCenter() { return center; }
    const Eigen::Vector3f& getNormal() const { return normal; }
    Eigen::Vector3f& getNormal() { return normal; }

    bool operator<(const SurfacePatch& other) const
    {
        return center.z() < other.center.z();
    }

    bool operator==(const SurfacePatch& other) const
    {
        return Base::operator ==(other) && center == other.center && normal == other.normal;
    }


protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SurfacePatchBase);
        ar & BOOST_SERIALIZATION_NVP(center);
        ar & BOOST_SERIALIZATION_NVP(normal);
    }
}; // SurfacePatch<MLSConfig::PRECALCULATED>

}  // namespace grid
}  // namespace maps


#endif /* __MAPS_SURFACEPATCHES_HPP_ */
