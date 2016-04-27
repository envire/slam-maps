#ifndef __MAPS_MLS_CONFIG_HPP__
#define __MAPS_MLS_CONFIG_HPP__

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

namespace maps { namespace grid
{

    /**
     * Configuration struct which hold information on the different 
     * options and parameters of the MLS. 
     */
    struct MLSConfig
    {
        MLSConfig()
        : gapSize( 1.0 )
        , thickness( 0.05 )
        , useColor( false )
        , updateModel( KALMAN )
        , useNegativeInformation( false )
        {}

        enum update_model
        {
            KALMAN,
            SLOPE
        };

        float gapSize;
        float thickness;
        bool useColor;
        update_model updateModel;
        bool useNegativeInformation;

    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(gapSize);
            ar & BOOST_SERIALIZATION_NVP(thickness);
            ar & BOOST_SERIALIZATION_NVP(useColor);
            ar & BOOST_SERIALIZATION_NVP(updateModel);
            ar & BOOST_SERIALIZATION_NVP(useNegativeInformation);
        }   
    };

}}

#endif // __MAPS_MLS_CONFIG_HPP__
