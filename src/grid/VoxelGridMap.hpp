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

#include "GridMap.hpp"
#include "DiscreteTree.hpp"

namespace maps { namespace grid
{

template<class CellT>
class VoxelGridMap : public GridMap< DiscreteTree<CellT> >
{
    typedef GridMap< DiscreteTree<CellT> > _Base;
public:
    VoxelGridMap(const Vector2ui &num_cells,
                const Eigen::Vector3d &resolution) :
                GridMap< DiscreteTree<CellT> >(num_cells,
                resolution.head<2>(), DiscreteTree<CellT>(resolution.z())) {}

    bool hasVoxelCell(const Eigen::Vector3d &position) const
    {
        Index idx;
        if(_Base::toGrid(position, idx))
        {
            return _Base::at(idx).hasCell(position.z());
        }
        return false;
    }

    bool hasVoxelCell(const Eigen::Vector3i &index) const
    {
        Index idx = index.head<2>();
        return _Base::at(idx).hasCell(index.z());
    }

    CellT& getVoxelCell(const Eigen::Vector3i &index)
    {
        Index idx = index.head<2>();
        return _Base::at(idx).getCellAt(index.z());
    }

    CellT& getVoxelCell(const Eigen::Vector3d &position)
    {
        return _Base::at(position)[position.z()];
    }

    bool fromVoxelGrid(const Eigen::Vector3i& idx, Eigen::Vector3d& position, bool checkIndex = true) const
    {
        Index idx_2d = idx.head<2>();
        if(_Base::fromGrid(idx_2d, position, checkIndex))
        {
            position << position.head<2>(), _Base::getDefaultValue().getCellCenter(idx.z());
            return true;
        }
        return false;
    }

    bool toVoxelGrid(const Eigen::Vector3d& position, Eigen::Vector3i& idx, bool checkIndex = true) const
    {
        Index idx_2d;
        if(_Base::toGrid(position, idx_2d, checkIndex))
        {
            idx << idx_2d, _Base::getDefaultValue().getCellIndex(position.z(), checkIndex);
            return true;
        }
        return false;
    }

    bool toVoxelGrid(const Eigen::Vector3d& position, Eigen::Vector3i& idx, Eigen::Vector3d &pos_diff) const
    {
        Index idx_2d;
        if(_Base::toGrid(position, idx_2d, pos_diff))
        {
            idx << idx_2d, _Base::at(idx_2d).getCellIndex(position.z(), pos_diff.z());
            return true;
        }
        return false;
    }

    Eigen::Vector3d getVoxelResolution() const
    {
        Eigen::Vector3d res;
        res << _Base::getResolution(), _Base::getDefaultValue().getResolution();
        return res;
    }

protected:

    /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridMap< DiscreteTree<CellT> >);
    }

};

}}
