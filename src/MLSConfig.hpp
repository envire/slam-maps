#ifndef __ENVIRE_MAPS_MLS_CONFIG_HPP__
#define __ENVIRE_MAPS_MLS_CONFIG_HPP__

namespace envire
{
    namespace maps 
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
                SUM,
                SLOPE
            };

            float gapSize;
            float thickness;
            bool useColor;
            update_model updateModel;
            bool useNegativeInformation;
        };

    }
}

#endif // __ENVIRE_MAPS_MLS_CONFIG_HPP__
