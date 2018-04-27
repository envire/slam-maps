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
#include "TraversabilityGrid.hpp"


using namespace maps::grid;

TraversabilityGrid::TraversabilityGrid() : GridMap<TraversabilityCell>()
{
}

TraversabilityGrid::TraversabilityGrid(const Vector2ui& num_cells, const Vector2d& resolution, const TraversabilityCell& default_value):
GridMap<TraversabilityCell>(num_cells, resolution, default_value)
{
}

TraversabilityGrid::TraversabilityGrid(const Vector2ui& num_cells, const Vector2d& resolution, const TraversabilityCell& default_value, const boost::shared_ptr< maps::LocalMapData >& data):
GridMap<TraversabilityCell>(num_cells, resolution, default_value, data)
{
}

TraversabilityGrid::~TraversabilityGrid()
{
}

bool TraversabilityGrid::setTraversabilityAndProbability(uint8_t traversabilityClassId, float probability, size_t x, size_t y)
{
    if (setProbability(probability, x, y) == false)
        return false;

    setTraversability(traversabilityClassId, x, y);
    return true;
}

void TraversabilityGrid::setTraversability(uint8_t traversabilityClassId, size_t x, size_t y)
{
    TraversabilityCell& cell = this->GridMap::at(x, y);
    cell.setTraversabilityClassId(traversabilityClassId);
}

const TraversabilityClass& TraversabilityGrid::getTraversability(size_t x, size_t y) const
{
    const TraversabilityCell& cell = this->GridMap::at(x, y);

    return traversabilityClasses[cell.getTraversabilityClassId()];
}

uint8_t TraversabilityGrid::getTraversabilityClassId(std::size_t x, std::size_t y) const
{
    const TraversabilityCell& cell = this->GridMap::at(x, y);

    return cell.getTraversabilityClassId();
}

bool TraversabilityGrid::setProbability(float probability, size_t x, size_t y)
{
    if (probability < 0 || probability > 1.0)
        return false;

    TraversabilityCell& cell = this->GridMap::at(x, y);
    uint8_t ui8probability = probability * std::numeric_limits<uint8_t>::max();
    cell.setProbability(ui8probability);
    return true;
}

float TraversabilityGrid::getProbability(size_t x, size_t y) const
{
    const TraversabilityCell& cell = this->GridMap::at(x, y);
    uint8_t ui8probability = cell.getProbability();
    float probability = static_cast<float>(ui8probability) / std::numeric_limits<uint8_t>::max();
    return probability;
}

void TraversabilityGrid::setTraversabilityClass(uint8_t traversabilityClassId, const TraversabilityClass& traversabilityClass)
{
    if(traversabilityClasses.size() <= traversabilityClassId)
        traversabilityClasses.resize(traversabilityClassId + 1);

    traversabilityClasses[traversabilityClassId] = traversabilityClass;
}

bool TraversabilityGrid::registerNewTraversabilityClass(uint8_t& retId, const TraversabilityClass& traversabilityClass)
{
    if (traversabilityClasses.size() > std::numeric_limits<uint8_t>::max())
        return false;
    retId = traversabilityClasses.size();
    setTraversabilityClass(retId, traversabilityClass);
    return true;
}

const TraversabilityClass& TraversabilityGrid::getTraversabilityClass(uint8_t traversabilityClassId) const
{
    return traversabilityClasses[traversabilityClassId];
}

const std::vector< TraversabilityClass >& TraversabilityGrid::getTraversabilityClasses() const
{
    return traversabilityClasses;
}
