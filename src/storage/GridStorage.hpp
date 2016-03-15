#pragma once

#include <vector>
#include <stdexcept>
#include "../Index.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

namespace envire {namespace maps
{
    
    template <typename T>
    class GridStorage
    {
        std::vector<T> cells;
        Vector2ui gridSize;
        T default_value;
    public:
        
        typedef typename std::vector<T>::iterator iterator;
        typedef typename std::vector<T>::const_iterator const_iterator;
        
        GridStorage(Vector2ui size, T default_value) : gridSize(size), default_value(default_value)
        {
            resize(size);
        }

        GridStorage(Vector2ui size) : GridStorage(size, T())
        {
        }

        GridStorage() : GridStorage(Vector2ui(0,0), T())
        {
        }

        GridStorage(const GridStorage &other) : cells(other.cells), gridSize(other.gridSize), default_value(other.default_value)
        {
        }

        const T &getDefaultValue() const
        {
            return default_value;
        }
        
        iterator begin()
        {
            return cells.begin();
        }

        iterator end()
        {
            return cells.end();
        }

        const_iterator begin() const
        {
            return cells.begin();
        }

        const_iterator end() const
        {
            return cells.end();
        }

        void resize(const Vector2ui &newSize)
        {
            gridSize = newSize;
            cells.resize(newSize.prod(), default_value);
        };

        void moveBy(const Vector2i &idx)
        {
            const Vector2ui num_cells(gridSize);
            
            // if all grid values should be moved outside
            if (abs(idx.x()) >= num_cells.x()
                || abs(idx.y()) >= num_cells.y())
            {
                clear();
                return;
            }

            std::vector<T> tmp;
            tmp.resize(gridSize.prod(), default_value);

            //copy pointers to new grid at new position
            for (unsigned int x = 0; x < num_cells.x(); ++x)
            {
                for (unsigned int y = 0; y < num_cells.y(); ++y)
                {
                    int x_new = x + idx.x();
                    int y_new = y + idx.y();

                    if ((x_new >= 0 && x_new < num_cells.x())
                        && (y_new >= 0 && y_new < num_cells.y()))
                    {
                        std::swap(cells[toIdx(x, y)], tmp[toIdx(x_new, y_new)]);
                    }
                }
            }
            
            cells.swap(tmp);
        }
        
        void moveBy(const Index &idx)
        {
            moveBy(Eigen::Vector2i(idx.x(), idx.y()));
        }
        
        const T& at(const Index &idx) const
        {
            if(idx.x() >= gridSize.x() || idx.y() >= gridSize.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[idx.x() + idx.y() * gridSize.x()];
        }

        T& at(const Index &idx)
        {
            if(idx.x() >= gridSize.x() || idx.y() >= gridSize.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[idx.x() + idx.y() * gridSize.x()];
        }

        const T& at(size_t x, size_t y) const
        {
            if(x >= gridSize.x() || y >= gridSize.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[x + y * gridSize.x()];
        }

        T& at(size_t x, size_t y)
        {
            if(x >= gridSize.x() || y >= gridSize.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[x + y * gridSize.x()];
        }
        
        const Vector2ui &getNumCells() const
        {
            return gridSize;
        };
        
        void clear()
        {
            for(T &e : cells)
            {
                e = default_value;
            }
        };
        
    protected:
        size_t toIdx(size_t x, size_t y) const
        {
            return x  +  y * gridSize.x();
        }

        /** Grants access to boost serialization */
        friend class boost::serialization::access;  

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(cells);
            ar & BOOST_SERIALIZATION_NVP(gridSize);
            ar & BOOST_SERIALIZATION_NVP(default_value);
        }          
    };
    
}}