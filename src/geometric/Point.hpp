#ifndef __MAPS_POINT_HPP__
#define __MAPS_POINT_HPP__

/** Boost serialization **/
#include <boost/serialization/nvp.hpp>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility>

namespace maps { namespace geometric
{
    /**@brief Point class IEEE 1873 standard
     * adapted to point in D-space.
     * T type (e.g. float or double)
     * D int specification for the dimensional space
     * **/
    template <typename T, int D>
    class Point: public Eigen::Matrix<T, D, 1, Eigen::DontAlign>
    {

    public:
        template <typename... Ts>
        Point(Ts&&... args) : Eigen::Matrix<T, D, 1, Eigen::DontAlign> (std::forward<Ts>(args)...)
        {
        }
    };

    typedef Point<double, 2> Point2d;
    typedef Point<double, 3> Point3d;
    typedef Point<float, 2> Point2f;
    typedef Point<float, 3> Point3f;
}}

namespace boost { namespace serialization
{
    template<class Archive, typename T, int D>
    void serialize(Archive & ar, ::maps::geometric::Point<T, D> & t, const unsigned int version)
    {
        for(size_t i=0; i<t.size(); i++)
            ar & BOOST_SERIALIZATION_NVP(t.data()[i]);
    }
}}

#endif /* __MAPS_POINT_HPP__ */
