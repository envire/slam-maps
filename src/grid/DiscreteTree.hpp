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
#pragma once

#include <map>
#include <cmath>
#include <limits>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/map.hpp>
#include <boost/format.hpp>

namespace maps { namespace grid
{

template<class S>
class DiscreteTree : public std::map<int32_t, S>
{
protected:
    typedef std::map<int32_t, S> TreeBase;
public:
    DiscreteTree(float resolution) : resolution(resolution) {}
    virtual ~DiscreteTree() {}

    S& operator[](float pos)
    {
        return TreeBase::operator[](getCellIndex(pos));
    }

    S& getCellAt(int32_t idx)
    {
        return TreeBase::operator[](idx);
    }

    const S& getCellAt(int32_t idx) const
    {
        return TreeBase::operator[](idx);
    }

    typename TreeBase::iterator find(const float pos)
    {
        return TreeBase::find(getCellIndex(pos));
    }

    typename TreeBase::const_iterator find(const float pos) const
    {
        return TreeBase::find(getCellIndex(pos));
    }

    typename TreeBase::iterator find(int32_t idx)
    {
        return TreeBase::find(idx);
    }

    typename TreeBase::const_iterator find(int32_t idx) const
    {
        return TreeBase::find(idx);
    }

    bool hasCell(float pos) const
    {
        return find(pos) != TreeBase::end();
    }

    bool hasCell(int32_t idx) const
    {
        return find(idx) != TreeBase::end();
    }

    float getCellCenter(int32_t idx) const
    {
        return (((float)idx) + 0.5f) * resolution;
    }

    int32_t getCellIndex(float pos, bool check_numeric_limits = false) const
    {
        if(check_numeric_limits && !isValidPos(pos))
            throw std::out_of_range((boost::format("DiscreteTree: The given value %1% is out of range!") % pos).str());
        return (int32_t)std::floor(pos / resolution);
    }

    int32_t getCellIndex(float pos, double z_diff) const
    {
        int32_t idx = getCellIndex(pos);
        z_diff = pos - getCellCenter(idx);
        return idx;
    }

    bool isValidPos(float pos) const
    {
        if(std::abs(pos) <= INT32_MAX * resolution)
            return true;
        return false;
    }

    float getResolution() const
    {
        return resolution;
    }

protected:
    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(TreeBase);
        ar & BOOST_SERIALIZATION_NVP(resolution);
    }

    float resolution;
};

}}
