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
#ifndef __MAPS_TAVERSABILITY_CLASS_HPP_
#define __MAPS_TAVERSABILITY_CLASS_HPP_



#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>

namespace maps { namespace grid 
{

    /**
     * @brief Type holding the actual traversability value used in TraversabilityGrid.
     **/
    class TraversabilityClass
    {
    public:

    /**
    * Default constructor.
    * Drivability must be given in the range of [0,1] (0 - 100%).
    * */
    TraversabilityClass(float drivability);

    TraversabilityClass();

    /**
    * Return whether this class is defined
    * or if it is an empty placeholder.
    * */
    bool isSet() const;

    /**
    * Returns whether the terrain is drivable.
    * */
    bool isTraversable() const;

    /**
    * Returns a value in the interval [0, 1].
    * Zero means not drivable at all and
    * one means perfect ground for driving.
    * */
    float getDrivability() const;

    private:
    float drivability;

    protected:

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {;
        ar & BOOST_SERIALIZATION_NVP(drivability);
    }
    };

}  //end namespace grid
}  //end namespace maps

#endif  //__MAPS_TAVERSABILITY_CLASS_HPP_
