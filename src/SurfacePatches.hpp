#ifndef __ENVIRE_MAPSC_SURFACEPATCHES_HPP_
#define __ENVIRE_MAPS_SURFACEPATCHES_HPP_

#include <numeric/PlaneFitting.hpp>
#include <boost/intrusive/list.hpp>

#include <Eigen/Core>
#include <base/Eigen.hpp>

#include "MLSConfig.hpp"


namespace envire {

namespace maps {

//! global typedef for all SurfacePatches. Only float seems to be useful.
typedef float SPScalar;
typedef Eigen::Matrix<SPScalar, 3, 1> Vector3;
typedef Eigen::Matrix<SPScalar, 3, 3> Matrix33;

/**
 * Struct to exchange surface patches. Stores minimal information, efficiently.
 * This struct does not allow to link patches together.
 */
struct SurfacePatchExchange
{
    Vector3 normal;
    SPScalar mean, height, sigma, mini, maxi;
};


typedef boost::intrusive::link_mode<boost::intrusive::auto_unlink> auto_unlink_mode;

class SurfacePatchBase : public boost::intrusive::list_base_hook<auto_unlink_mode>
{
protected:
    size_t update_idx;
    float min, max;
public:
    explicit SurfacePatchBase(const float &z, const size_t& updateIdx = 0)
    : update_idx(updateIdx)
    , min(z), max(z)
    { }

    void merge(const SurfacePatchBase& other)
    {
        min = std::min(min, other.min);
        max = std::max(max, other.max);
        update_idx = std::max(update_idx, other.update_idx);
    }

    float getMin() const { return min; }
    float getMax() const { return max; }
    float getTop() const { return max; }
    float getBottom() const { return min; }


};


/**
 * templated surface patch class.
 * Eventually, this should be renamed to SurfacePatch
 * TODO add color option, write implementations for all variants
 */
template<MLSConfig::update_model model>
class SurfacePatchT;


/**
 * SurfacePatch type for SLOPE update model.
 */
template<>
class SurfacePatchT<MLSConfig::SLOPE> : SurfacePatchBase
{
    base::PlaneFitting<float> plane;
    typedef SurfacePatchBase Base;
public:
    SurfacePatchT(const Eigen::Vector3f& point, const float& cov)
: Base(point.z())
, plane(point, 1.0f/cov)
{ }

    /**
     * Compares two patches by their center of mass
     */
    bool operator<(const SurfacePatchT& other) const
    {
        return plane.z * other.plane.n < other.plane.z * plane.n;
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

    Eigen::Vector3f getNormal() const
    {
        return Eigen::Vector3f::UnitZ();
    }
};


//        template<MLSConfig::update_model model>
//    class SurfacePatchBase : public


}  // namespace maps

}  // namespace envire



#endif /* __ENVIRE_MAPS_SURFACEPATCHES_HPP_ */
