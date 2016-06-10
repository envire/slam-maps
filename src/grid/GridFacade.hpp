#pragma once

#include "GridAccessInterface.hpp"

namespace maps { namespace grid
{
    template <typename CellBaseT>
    class GridFacade
    {
        GridAccessInterface<CellBaseT> *impl;
    public:
        
        typedef AccessIterator<CellBaseT> iterator;
        typedef ConstAccessIterator<CellBaseT> const_iterator;
        
        GridFacade(GridAccessInterface<CellBaseT> *impl) : impl(impl)
        {
        }

        const CellBaseT &getDefaultValue() const
        {
            return impl->getDefaultValue();
        }
        
        iterator begin()
        {
            return impl->begin();
        }

        iterator end()
        {
            return impl->end();
        }

        const_iterator begin() const
        {
            return impl->begin();
        }

        const_iterator end() const
        {
            return impl->end();
        }

        void resize(const Index &new_number_cells)
        {
            impl->resize(new_number_cells);
        };
        
        void moveBy(const Index &idx)
        {
            impl->moveBy(idx);
        }

        void moveBy(const Eigen::Vector2i &idx)
        {
//             impl->moveBy(idx);
        }
        
        const CellBaseT& at(const Index &idx) const
        {
            return impl->at(idx);
        }

        CellBaseT& at(const Index &idx)
        {
            return impl->at(idx);
        }

        const CellBaseT& at(size_t x, size_t y) const
        {
            return impl->at(Index(x,y));
        }

        CellBaseT& at(size_t x, size_t y)
        {
            return impl->at(Index(x,y));
        }
        
        const Index &getNumCells() const
        {
            return impl->getNumCells();
        };
        
        void clear()
        {
            impl->clear();
        };
    };
    
}}
