#pragma once

#include "GridAccessInterface.hpp"

namespace maps { namespace grid
{
    template <typename CellT, typename CellBaseT>
    class VectorGridAccess : public GridAccessInterface<CellBaseT>
    {
        VectorGrid<CellT> *grid;
    public:
        VectorGridAccess(VectorGrid<CellT> *grid) : grid(grid)
        {
        }

        virtual ~VectorGridAccess()
        {
        }

        virtual const CellBaseT &getDefaultValue() const
        {
            return grid->getDefaultValue();
        }

        virtual typename GridAccessInterface<CellBaseT>::iterator begin()
        {
            return AccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->begin());
        }

        virtual typename GridAccessInterface<CellBaseT>::iterator end()
        {
            return AccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->end());
        }

        virtual typename GridAccessInterface<CellBaseT>::const_iterator begin() const
        {
            return ConstAccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->begin());
        }

        virtual typename GridAccessInterface<CellBaseT>::const_iterator end() const
        {
            return ConstAccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->end());
        }

        virtual void resize(Index new_number_cells)
        {
            grid->resize(new_number_cells);
        };

        virtual void moveBy(Index idx)
        {
            grid->moveBy(idx);
        }

        virtual const CellBaseT& at(Index idx) const
        {
            return static_cast<const CellBaseT &>(grid->at(idx));
        }

        virtual CellBaseT& at(Index idx)
        {
            return static_cast<CellBaseT &>(grid->at(idx));
        }

        virtual const CellBaseT& at(size_t x, size_t y) const
        {
            return static_cast<const CellBaseT &>(grid->at(x, y));
        }

        virtual CellBaseT& at(size_t x, size_t y)
        {
            return static_cast<CellBaseT &>(grid->at(x, y));
        }

        virtual const Index &getNumCells() const
        {
            return grid->getNumCells();
        };

        virtual void clear()
        {
            grid->clear();
        };
    };
}}
