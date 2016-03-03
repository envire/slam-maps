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

        void resize(const Vector2ui &newSize)
        {
            impl->resize(newSize);
        };
        
        void moveBy(const Index &idx)
        {
            impl->moveBy(idx);
        }

        void moveBy(const Eigen::Vector2i &idx)
        {
//             impl->moveBy(idx);
        }
        
        const T& at(const Index &idx) const
        {
            return impl->at(idx);
        }

        T& at(const Index &idx)
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