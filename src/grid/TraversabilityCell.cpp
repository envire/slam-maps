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
#include "TraversabilityCell.hpp"


using namespace maps::grid;

TraversabilityCell::TraversabilityCell(): traversabilityClassId(0), probability(0)
{
}

TraversabilityCell::TraversabilityCell(uint8_t traversabilityClass, uint8_t probability): traversabilityClassId(traversabilityClass), probability(probability)
{
}

TraversabilityCell::~TraversabilityCell()
{
}

void TraversabilityCell::setTraversabilityClassId(uint8_t traversabilityClassId)
{
    this->traversabilityClassId = traversabilityClassId;
}

uint8_t TraversabilityCell::getTraversabilityClassId() const
{
    return traversabilityClassId;
}

void TraversabilityCell::setProbability(uint8_t probability)
{
    this->probability = probability;
}

uint8_t TraversabilityCell::getProbability() const
{
    return probability;
}

bool TraversabilityCell::operator==(const TraversabilityCell& other) const
{
    bool isequal = traversabilityClassId == other.getTraversabilityClassId() && probability == other.getProbability();
    return isequal;
}

bool TraversabilityCell::operator!=(const TraversabilityCell& other) const
{
    bool isinequal = traversabilityClassId != other.getTraversabilityClassId() || probability != other.getProbability();
    return isinequal;
}
