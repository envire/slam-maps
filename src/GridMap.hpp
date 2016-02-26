#ifndef __ENVIRE_MAPS_GRID_MAP_HPP__
#define __ENVIRE_MAPS_GRID_MAP_HPP__

#include "Grid.hpp"

/** std **/
#include <iostream>
#include <type_traits>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

/** Boost **/
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>

#include "AccessIterator.hpp"


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

        void resize(Vector2ui newSize)
        {
            gridSize = newSize;
            cells.resize(newSize.prod(), default_value);
        };
        
        void moveBy(Index idx)
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
        
        const T& at(Index idx) const
        {
            if(idx.x() >= gridSize.x() || idx.y() >= gridSize.y())
                throw std::runtime_error("Provided index is out of the grid");
            return cells[idx.x() + idx.y() * gridSize.x()];
        }

        T& at(Index idx)
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
    };

    
    
    template <typename T>
    class GridStorageAccess
    {
    public:
        
        typedef AccessIterator<T> iterator;
        typedef ConstAccessIterator<T> const_iterator;
        
        virtual const T &getDefaultValue() const
        {
            throw std::runtime_error("Full virtual method called");
        }
        
        virtual iterator begin()
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual iterator end()
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual const_iterator begin() const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual const_iterator end() const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual void resize(Vector2ui newSize)
        {
            throw std::runtime_error("Full virtual method called");
        };
        
        virtual void moveBy(Index idx)
        {
            throw std::runtime_error("Full virtual method called");
        }
        
        virtual const T& at(Index idx) const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual T& at(Index idx)
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual const T& at(size_t x, size_t y) const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual T& at(size_t x, size_t y)
        {
            throw std::runtime_error("Full virtual method called");
        }
        
        virtual const Vector2ui &getNumCells() const
        {
            throw std::runtime_error("Full virtual method called");
        };
        
        virtual void clear()
        {
            throw std::runtime_error("Full virtual method called");
        };
    };
    
    template <typename T, typename TBASE>
    class GridStorageAccessImpl : public GridStorageAccess<TBASE>
    {
        GridStorage<T> *storage;
    public:
        GridStorageAccessImpl(GridStorage<T> *storage) : storage(storage)
        {
        }

        virtual const TBASE &getDefaultValue() const
        {
            return storage->getDefaultValue();
        }
        
        virtual typename GridStorageAccess<TBASE>::iterator begin()
        {
            return AccessIteratorImpl<T, TBASE, std::vector<T> >(storage->begin());
        }

        virtual typename GridStorageAccess<TBASE>::iterator end()
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual typename GridStorageAccess<TBASE>::const_iterator begin() const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual typename GridStorageAccess<TBASE>::const_iterator end() const
        {
            throw std::runtime_error("Full virtual method called");
        }

        virtual void resize(Vector2ui newSize)
        {
            storage->resize(newSize);
        };
        
        virtual void moveBy(Index idx)
        {
            storage->moveBy(idx);
        }
        
        const TBASE& at(Index idx) const
        {
            return static_cast<const TBASE &>(storage->at(idx));
        }

        TBASE& at(Index idx)
        {
            return static_cast<TBASE &>(storage->at(idx));
        }

        const TBASE& at(size_t x, size_t y) const
        {
            return static_cast<const TBASE &>(storage->at(x, y));
        }

        TBASE& at(size_t x, size_t y)
        {
            return static_cast<TBASE &>(storage->at(x, y));
        }
        
        const Vector2ui &getNumCells() const
        {
            return storage->getNumCells();
        };
        
        void clear()
        {
            storage->clear();
        };
    };

    
    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename T, typename R = GridStorage<T> >
    class GridMap: public Grid, public R
    {

    protected:

    public:
        GridMap() 
            : Grid()
        {
        }

        GridMap(const Grid& oGrid, R oStorage)
            : Grid(oGrid), R(oStorage)
        {
        }

        GridMap(const GridMap& other)
            : Grid(other), R(other)
        {
        }
        
        /** @brief Constructor of the abstract GridMap class
         *
         * Defines the extends and positioning of the grid. The grid is assumed
         * to be on the x-y plane of the reference frame. The number of grid
         * cells is given by the num_cell_x and num_cell_y params. Each dimension
         * also has an scaling and offset parameter, such that the origin of the
         * grid can be moved around and the grid scaled.
         *
         * The relation between the grid cell index and position is:
         *
         * @verbatim
         *
         * v_position = v_index * resolution + offset
         *
         * where:
         * v_position is a 2x1 vector [x, y] wrt the local coordinate frame
         * v_index is a 2x1 vector [x, y] defining the index in the grid
         * resolution is a 2x1 vector defining the resolution of each grid cell
         * offset as it is defined in LocalMap class
         *
         * @endverbatim
         *
         * @param resolution - resolution of the x and y axis
         * @param num_cell - number of cells in x and y direction
         * @param default_value - default value
         */
        GridMap(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const T& default_value)
            : Grid(num_cells, resolution),
              R(num_cells, default_value)
        {
        }

        GridMap(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const T& default_value,
                const boost::shared_ptr<LocalMapData> &data)
            : Grid(num_cells, resolution, data),
              R(num_cells, default_value)
        {}

        /** @brief default destructor
         */
        ~GridMap()
        {
        }

    public:
        const T& at(const Vector3d& pos) const
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid.");
            return at(idx);
        }

        T& at(const Vector3d& pos)
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid");
            return at(idx);
        }

        using R::at;
        
        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (integral and floating types)
         * hide this function e.g. for class types
         * @return [description]
         */
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMax() const
        {
            Vector2ui numCells(getNumCells());
            
            std::cout << "Num Cells is " << numCells.transpose() << std::endl;
            if(numCells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute max on empty map");

            
            
            return *std::max_element(this->begin(), this->end());
        }

        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (integral and floating types)
         * hide this function e.g. for class types
         * @return [description]
         */
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMin() const
        {
            Vector2ui numCells(getNumCells());
            
            if(numCells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute min on empty map");

            return *std::min_element(this->begin(), this->end());
        }
        
        virtual const Vector2ui& getNumCells() const
        {
            return R::getNumCells();
        };

    protected:
    };
    
}}
#endif // __ENVIRE_MAPS_GRID_MAP_HPP__
