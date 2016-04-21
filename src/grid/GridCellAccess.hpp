#pragma once

#include "GridCellAccessInterface.hpp"


namespace maps { namespace grid
{
    template <typename T, typename TBASE>
    class GridCellAccess : public GridCellAccessInterface<TBASE>
    {
        GridCell<T> *storage;
    public:
        GridCellAccess(GridCell<T> *storage) : storage(storage)
        {
        }

        virtual const TBASE &getDefaultValue() const
        {
            return storage->getDefaultValue();
        }
        
        virtual typename GridCellAccessInterface<TBASE>::iterator begin()
        {
            return AccessIteratorImpl<T, TBASE, std::vector<T> >(storage->begin());
        }

        virtual typename GridCellAccessInterface<TBASE>::iterator end()
        {
            return AccessIteratorImpl<T, TBASE, std::vector<T> >(storage->end());
        }

        virtual typename GridCellAccessInterface<TBASE>::const_iterator begin() const
        {
            return ConstAccessIteratorImpl<T, TBASE, std::vector<T> >(storage->begin());
        }

        virtual typename GridCellAccessInterface<TBASE>::const_iterator end() const
        {
            return ConstAccessIteratorImpl<T, TBASE, std::vector<T> >(storage->end());
        }

        virtual void resize(Vector2ui new_number_cells)
        {
            storage->resize(new_number_cells);
        };
        
        virtual void moveBy(Index idx)
        {
            storage->moveBy(idx);
        }
        
        virtual const TBASE& at(Index idx) const
        {
            return static_cast<const TBASE &>(storage->at(idx));
        }

        virtual TBASE& at(Index idx)
        {
            return static_cast<TBASE &>(storage->at(idx));
        }

        virtual const TBASE& at(size_t x, size_t y) const
        {
            return static_cast<const TBASE &>(storage->at(x, y));
        }

        virtual TBASE& at(size_t x, size_t y)
        {
            return static_cast<TBASE &>(storage->at(x, y));
        }
        
        virtual const Vector2ui &getNumCells() const
        {
            return storage->getNumCells();
        };
        
        virtual void clear()
        {
            storage->clear();
        };
    };    
}}
