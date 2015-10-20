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
        struct MLSConfiguration
        {
            MLSConfiguration()
        	: gapSize( 1.0 ), 
        	thickness( 0.05 ),
        	useColor( false ),
        	updateModel( KALMAN ) {}

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
        };

    }
}

#endif // __ENVIRE_MAPS_MLS_CONFIG_HPP__
