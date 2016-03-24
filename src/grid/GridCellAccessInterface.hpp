#pragma once

#include "AccessIterator.hpp"
#include "../Index.hpp"

namespace maps
{
    template <typename T>
    class GridCellAccessInterface
    {
    public:
        
        typedef AccessIterator<T> iterator;
        typedef ConstAccessIterator<T> const_iterator;
        
        GridCellAccessInterface()
        {
        }

        virtual const T &getDefaultValue() const = 0;
        
        virtual iterator begin() = 0;

        virtual iterator end() = 0;

        virtual const_iterator begin() const = 0;

        virtual const_iterator end() const = 0;

        virtual void resize(Vector2ui newSize) = 0;
        
        virtual void moveBy(Index idx) = 0;
        
        virtual const T& at(Index idx) const = 0;

        virtual T& at(Index idx) = 0;

        virtual const Vector2ui &getNumCells() const = 0;
        
        virtual void clear() = 0;
    };
}
