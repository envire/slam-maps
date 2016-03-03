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
    size_t update_idx; // TODO is this really useful?
    float min, max;
public:
    // TODO TYPE is for compatibility with old Patches -- probably should be made optional
    enum TYPE
    {
        VERTICAL = 0,
        HORIZONTAL = 1,
        NEGATIVE = 2
    };

    explicit SurfacePatchBase(const float &z, const size_t& updateIdx = 0)
    : update_idx(updateIdx)
    , min(z), max(z)
    { }

    explicit SurfacePatchBase(const float &mean, const float &height, const size_t& updateIdx = 0)
    : update_idx(updateIdx)
    , min(mean - height), max(mean)
    { }

    bool merge(const SurfacePatchBase& other, const float& gapSize=0.0f)
    {
        if(!isCovered(other, gapSize)) return false;
        min = std::min(min, other.min);
        max = std::max(max, other.max);
        update_idx = std::max(update_idx, other.update_idx);
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
        // This is equivalent to the old overlap function.
        // TODO Some overlapping cases are not covered by this!
        return
//                (min - gapSize < other.max && max + gapSize > other.max) ||
//                (min - gapSize < other.min && max + gapSize > other.min);
                isCovered(other.max, gapSize) || isCovered(other.min, gapSize);

    }

    bool isNegative() const
    {
        return false; // base patch does not allow negative patches
    }


};

/**
 * Templated surface patch class.
 * Eventually, this should be renamed to SurfacePatch
 * TODO add color option, write implementations for all variants
 */
template<MLSConfig::update_model model>
class SurfacePatchT;


/**
 * SurfacePatch type for SLOPE update model.
 */
template<>
class SurfacePatchT<MLSConfig::SLOPE> : public SurfacePatchBase
{
    typedef SurfacePatchBase Base;
    base::PlaneFitting<float> plane;
    float n;
    TYPE type;
public:
    SurfacePatchT(const Eigen::Vector3f& point, const float& cov)
        : Base(point.z())
        , plane(point, 1.0f/cov)
        , n(1), type(TYPE::HORIZONTAL)
    { }

    SurfacePatchT(const float& mean, const float& stdev, const float& height = 0, TYPE type_=TYPE::HORIZONTAL)
        : Base(mean, height)
        , n(1), type(type_)
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

    Eigen::Vector3f getCenter() const
    {
        Eigen::Vector3f center(plane.x, plane.y, plane.z);
        return center * (1.0f/plane.n);
    }

    Eigen::Vector3f getNormal() const
    {
        if(n<=1.0f) return Eigen::Vector3f::UnitZ();
        typedef base::PlaneFitting<float>::Matrix3 Mat3;
        Mat3 moments;
        moments << plane.xx, plane.xy, plane.xz,
                   plane.xy, plane.yy, plane.yz,
                   plane.xz, plane.yz, plane.zz;
        moments *= 1.0/plane.n;
        Eigen::Vector3f mu = getCenter();

        moments -= mu * mu.transpose();
        Eigen::SelfAdjointEigenSolver<Mat3> eig;
        eig.computeDirect(moments, Eigen::ComputeEigenvectors);
        return eig.eigenvectors().col(0);
    }

    bool merge(const SurfacePatchT& other, const MLSConfig& config)
    {
        if(Base::merge(other, config.gapSize))
        {
            plane.update(other.plane);
            n+= other.n;
            return true;
        }

        return false;
    }

}; // SurfacePatchT<MLSConfig::SLOPE>





template<>
class SurfacePatchT<MLSConfig::KALMAN>: public SurfacePatchBase
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
    SurfacePatchT(const Eigen::Vector3f& point, const float& cov)
        : Base(point.z())
        , mean(point.z()), var(cov), height(0)
    { }

    SurfacePatchT(const float& mean, const float& stdev, const float& height = 0, TYPE type_=TYPE::HORIZONTAL)
        : Base(mean, height)
        , mean(mean), var(stdev*stdev), height(height)
    { }
    bool merge(const SurfacePatchT& other, const MLSConfig& config)
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

    bool operator<(const SurfacePatchT& other) const
    {
        return mean < other.mean;
    }

    Eigen::Vector3f getNormal() const
    {
        return Eigen::Vector3f::UnitZ();
    }




}; // SurfacePatchT<MLSConfig::KALMAN>



}  // namespace maps

}  // namespace envire



#endif /* __ENVIRE_MAPS_SURFACEPATCHES_HPP_ */
