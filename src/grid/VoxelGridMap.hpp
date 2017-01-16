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
                resolution.block(0,0,2,1), DiscreteTree<CellT>(resolution.z())) {}

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
        Index idx = index.block(0,0,2,1);
        return _Base::at(idx).hasCell(index.z());
    }

    CellT& getVoxelCell(const Eigen::Vector3i &index)
    {
        Index idx = index.block(0,0,2,1);
        return _Base::at(idx).getCellAt(index.z());
    }

    CellT& getVoxelCell(const Eigen::Vector3d &position)
    {
        return _Base::at(position)[position.z()];
    }

    bool fromVoxelGrid(const Eigen::Vector3i& idx, Eigen::Vector3d& position, bool checkIndex = true) const
    {
        Index idx_2d = idx.block(0,0,2,1);
        if(_Base::fromGrid(idx_2d, position, checkIndex))
        {
            position << position.block(0,0,2,1), _Base::getDefaultValue().getCellCenter(idx.z());
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