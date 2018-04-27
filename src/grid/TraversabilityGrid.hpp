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
#ifndef __MAPS_TAVERSABILITY_GRID_HPP_
#define __MAPS_TAVERSABILITY_GRID_HPP_


#include "GridMap.hpp"
#include "TraversabilityCell.hpp"
#include "TraversabilityClass.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>

namespace maps { namespace grid
{
    /**
    * @brief Maptype to represent the ability of the robot to traverse different
    * parts of the terrain.
    * @details The grid holds TraversabilityCells which in turn hold information
    * on the traversability (how well can I drive here?) and probability
    * (uncertainty of the data for this point). To conserve storage space both
    * values are stored in the grid as an uint8_t. For the traversability this
    * represents the Id of a TraversabilityClass in traversabilityClasses which
    * holds the actual value. The probability is simply converted from float [0, 1]
    * to uint8_t  [0, 255].
    * */
    class TraversabilityGrid : public GridMap<TraversabilityCell>
    {
    public:
        enum Classes {
                CLASS_UNKNOWN = 0,
                CLASS_OBSTACLE = 1,
                CUSTOM_CLASSES = 2
        };

    private:
        std::vector<TraversabilityClass> traversabilityClasses;

    public:
        TraversabilityGrid();

        // Use this if you want a map holding individual data.
        TraversabilityGrid(const Vector2ui &num_cells, const Vector2d &resolution, const TraversabilityCell &default_value);

        // Use this if the map is supposed to hold data shared with other maps.
        TraversabilityGrid(const Vector2ui &num_cells, const Vector2d &resolution, const TraversabilityCell &default_value, const boost::shared_ptr<LocalMapData> &data);

        ~TraversabilityGrid();

        // Set the traversabilityClassId and probability at the given coordinates.
        // Note that probability has to be in range [0,1].
        bool setTraversabilityAndProbability(uint8_t traversabilityClassId, float probability, size_t x, size_t y);

        void setTraversability(uint8_t traversabilityClassId, size_t x, size_t y);
        const TraversabilityClass& getTraversability(size_t x, size_t y) const;

        // Returns the traversabilityClassId that is stored in the TraversabilityCell at the given coordinates.
        uint8_t getTraversabilityClassId(size_t x, size_t y) const;

        // Note that probability has to be in range [0,1].
        bool setProbability(float probability, size_t x, size_t y);
        float getProbability(size_t x, size_t y) const;

        // Set the traversabilityClass for the given traversabilityClassId.
        // Note that only IDs in the range [0,255] will be accepted.
        void setTraversabilityClass(uint8_t traversabilityClassId, const maps::grid::TraversabilityClass& traversabilityClass);

        // Add a new TraversabilityClass to traversabilityClasses.
        bool registerNewTraversabilityClass(uint8_t& retId, const maps::grid::TraversabilityClass& traversabilityClass);

        const TraversabilityClass &getTraversabilityClass(uint8_t traversabilityClassId) const;
        const std::vector<TraversabilityClass> &getTraversabilityClasses() const;

    protected:

        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridMap<TraversabilityCell>);
        ar & BOOST_SERIALIZATION_NVP(traversabilityClasses);
        }
    };


}  //namespace grid
}  //namespace maps

#endif  // __MAPS_TAVERSABILITY_GRID_HPP_
