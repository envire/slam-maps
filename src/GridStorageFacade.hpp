#pragma once

#include "GridStorageInterface.hpp"

namespace envire {namespace maps
{
    template <typename T>
    class GridStorageFacade
    {
        GridStorageInterface<T> *impl;
    public:
        
        typedef AccessIterator<T> iterator;
        typedef ConstAccessIterator<T> const_iterator;
        
        GridStorageFacade(GridStorageInterface<T> *impl) : impl(impl)
        {
        }

        const T &getDefaultValue() const
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

        void resize(Vector2ui newSize)
        {
            impl->resize(newSize);
        };
        
        void moveBy(Index idx)
        {
            impl->moveBy(idx);
        }
        
        const T& at(Index idx) const
        {
            return impl->at(idx);
        }

        T& at(Index idx)
        {
            return impl->at(idx);
        }

        const T& at(size_t x, size_t y) const
        {
            return impl->at(Index(x,y));
        }

        T& at(size_t x, size_t y)
        {
            return impl->at(Index(x,y));
        }
        
        const Vector2ui &getNumCells() const
        {
            return impl->getNumCells();
        };
        
        void clear()
        {
            impl->clear();
        };
    };
    
}}