#ifndef __MAPS_MLS_CONFIG_HPP__
#define __MAPS_MLS_CONFIG_HPP__

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
    };

}}

#endif // __MAPS_MLS_CONFIG_HPP__
