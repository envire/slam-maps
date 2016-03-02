#pragma once

#include "GridStorageInterface.hpp"


namespace envire {namespace maps
{
    template <typename T, typename TBASE>
    class GridStorageAccess : public GridStorageInterface<TBASE>
    {
        GridStorage<T> *storage;
    public:
        GridStorageAccess(GridStorage<T> *storage) : storage(storage)
        {
        }

        virtual const TBASE &getDefaultValue() const
        {
            return storage->getDefaultValue();
        }
        
        virtual typename GridStorageInterface<TBASE>::iterator begin()
        {
            return AccessIteratorImpl<T, TBASE, std::vector<T> >(storage->begin());
        }

        virtual typename GridStorageInterface<TBASE>::iterator end()
        {
            return AccessIteratorImpl<T, TBASE, std::vector<T> >(storage->end());
        }

        virtual typename GridStorageInterface<TBASE>::const_iterator begin() const
        {
            return ConstAccessIteratorImpl<T, TBASE, std::vector<T> >(storage->begin());
        }

        virtual typename GridStorageInterface<TBASE>::const_iterator end() const
        {
            return ConstAccessIteratorImpl<T, TBASE, std::vector<T> >(storage->end());
        }

        virtual void resize(Vector2ui newSize)
        {
            storage->resize(newSize);
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