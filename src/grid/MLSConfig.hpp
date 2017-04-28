//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
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
            , PRECALCULATED //! Patches with precalculated normal vector (can't be updated)
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
