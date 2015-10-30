#ifndef __ENVIRE_MAPS_GRID_HPP__
#define __ENVIRE_MAPS_GRID_HPP__

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>

#include "GridBase.hpp"

namespace envire 
{
    namespace maps 
    {
        template <typename T>
        class Grid : public GridBase {

        public:
            typedef boost::multi_array<T,2> ArrayType;

            Grid()
                :GridBase(),
                holder(new ArrayType())
            {
            }

            Grid(const T& default_value, GridConfig config)
                : GridBase(config),
                holder(new ArrayType()),
                default_value(default_value)
            {
            }

            Grid(const Grid& other) :
                GridBase(other.config),
                default_value(other.default_value),
                holder(new ArrayType(*(other.holder)))
            {
            } 

            ~Grid() 
            {
                delete holder;
            }

            const T& getDefaultValue() const 
            {
                return default_value;
            }

            const T& at(const Eigen::Vector2d& pos) const
            {
                Index idx;
                if (!toGrid(pos, idx))
                    throw std::runtime_error("Provided position is out of the grid.");
                return get(idx);
            }       

            T& at(const Eigen::Vector2d& pos)
            {
                Index idx;
                if (!toGrid(pos, idx))
                    throw std::runtime_error("Provided position is out of the grid");
                return get(idx);
            }                   

            const T& at(Index idx) const
            {
                if (!inGrid(idx))
                    throw std::runtime_error("Provided index is out of the grid");              
                return get(idx);
            }   

            T& at(Index idx)
            {
                if (!inGrid(idx))
                    throw std::runtime_error("Provided index is out of the grid");              
                return get(idx);
            }

            const T& at(size_t x, size_t y) const
            {
                Index idx(x, y);
                if (!inGrid(idx))
                    throw std::runtime_error("Provided index is out of the grid");              
                return get(idx);                
            }               

            T& at(size_t x, size_t y)
            {
                Index idx(x, y);
                if (!inGrid(idx))
                    throw std::runtime_error("Provided index is out of the grid");              
                return get(idx);                
            }               

            const T& getMax() const
            {
                const ArrayType &array = getArray();
                // TODO: if max_element return end iterator
                return *(std::max_element(array.origin(), array.origin() + array.num_elements()));
            }

            const T& getMin() const
            {
                const ArrayType &array = getArray();
                // TODO: if min_element return end iterator
                return *(std::min_element(array.origin(), array.origin() + array.num_elements()));
            }           

            void moveBy(Index idx) 
            {
                // if all grid values should be moved outside
                if (abs(idx.x) >= getCellSizeX()
                    || abs(idx.y) >= getCellSizeY())
                {
                    init();
                    return;
                }

                ArrayType &src = getArray();

                ArrayType tmp;
                tmp.resize(boost::extents[getCellSizeY()][getCellSizeX()]);
                std::fill(tmp.data(), 
                            tmp.data() + tmp.num_elements(), 
                            default_value);         

                boost::swap(tmp, src);
                
                for (int x = 0; x < getCellSizeX(); ++x)
                {
                    for (int y = 0; y < getCellSizeY(); ++y)
                    {
                        int x_new = x + idx.x;
                        int y_new = y + idx.y;

                        if (x_new >= 0 && x_new < getCellSizeX() 
                            && y_new >= 0 && y_new < getCellSizeY())
                        {
                            get(Index(x_new, y_new)) = *(tmp.data() + y * getCellSizeX() + x); 
                        }
                    }
                }
            }

            void clear() 
            {
                init();
            }

        private:
            ArrayType* holder;  
            T default_value;

            T& get(Index idx)
            {
                return *(getArray().data() + idx.y * getCellSizeX() + idx.x);         
            }

            const T& get(Index idx) const
            {
                return *(getArray().data() + idx.y * getCellSizeX() + idx.x);      
            }           

            void init() const
            {           
                holder->resize(boost::extents[getCellSizeY()][getCellSizeX()]);
                std::fill(holder->data(), 
                            holder->data() + holder->num_elements(), 
                            default_value);     
            }       

            ArrayType& getArray()
            {
                if (holder->num_elements() == 0)
                    init();

                return *holder;
            }

            const ArrayType& getArray() const
            {       
                if (holder->num_elements() == 0)
                    init();             

                return *holder;
            }               
        };
    }
}

#endif // __ENVIRE_MAPS_GRID_HPP__