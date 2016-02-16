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

namespace envire {namespace maps
{

    class GridElementFactory
    {
    public:
        virtual void *getNewInstance() const = 0;
        
        virtual void setDefault(void *e) const = 0;
        
        virtual void deleteInstance(void *e) const = 0;
        
        virtual const void *getDefaultValue() const = 0;
    };

    template <typename T>
    class GridElementFactoryImpl : public GridElementFactory
    {
        /** default value type **/
        T default_value;
    public:
        GridElementFactoryImpl(T defaultVal) : default_value(defaultVal)
        {
        }
        
        GridElementFactoryImpl()
        {
        }
        
        virtual const void *getDefaultValue() const
        {
            return &default_value;
        }
        
        virtual void *getNewInstance() const
        {
            return new T(default_value);
        }

        virtual void setDefault(void *v) const
        {
            *static_cast<T *>(v) = default_value;
        }
        
        virtual void deleteInstance(void* e) const
        {
            delete static_cast<T *>(e);
        }
    };

    class GridStorageBase
    {
    public:
        typedef boost::multi_array<void *, 2> GridCell;

        GridStorageBase(GridStorageBase &other) : cells(other.cells), factory(other.factory)
        {
            //TODO change to copy case
        }

        GridStorageBase(const GridStorageBase &other) : cells(other.cells), factory(other.factory)
        {
            //TODO change to copy case
        }


        GridStorageBase(boost::shared_ptr<GridCell> c, boost::shared_ptr<GridElementFactory> f) : cells(c), factory(f)
        {
        }
        
        GridStorageBase(GridElementFactory *fac)
        {
            factory.reset(fac);
            cells.reset(new GridCell());
        }

    protected:
        /** Store the actual content of the cells of the grid **/
        boost::shared_ptr<GridCell> cells;
        
        boost::shared_ptr<GridElementFactory> factory;
    };
    
    template <typename T>
    class GridStorage : public GridStorageBase
    {
    public:
        GridStorage(Vector2ui size, T default_value) : GridStorageBase(new GridElementFactoryImpl<T>(default_value))
        {
            resize(size);
        }

        GridStorage(Vector2ui size) : GridStorage(size, T())
        {
            resize(size);
        }

        GridStorage(GridStorage &other) : GridStorageBase(other)
        {
        }

        GridStorage(const GridStorage &other) : GridStorageBase(other)
        {
            //FIXME create copy
        }

        template <typename TY>
        GridStorage<TY> *cast()
        {
            static_assert(
                std::is_base_of<TY, T>::value, 
                "T must be a descendant of TBASE"
            );

            return reinterpret_cast<GridStorage<TY> *>(this);
        };

        const T &getDefaultValue() const
        {
            return *static_cast<const T *> (factory->getDefaultValue());
        }
//         GridCell& getCells()
//         {
//             return *cells;
//         }
// 
//         const GridCell& getCells() const
//         {
//             return *cells;
//         }
//         
        void resize(Vector2ui newSize)
        {
            Index oldIndex(getNumCells());
            
            cells->resize(boost::extents[newSize.x()][newSize.y()]);
            
            for (unsigned int x = 0; x < newSize.x(); ++x)
            {
                for (unsigned int y = 0; y < newSize.y(); ++y)
                {
                    if(x >= oldIndex.x() 
                        && y >= oldIndex.y())
                    {
                        void *newElem = factory->getNewInstance();
                        factory->setDefault(newElem);
                        (*cells)[x][y] = newElem;
                    }
                }
            }
            
        };
        
        void moveBy(Index idx)
        {
            const Vector2ui num_cells(getNumCells());
            
            // if all grid values should be moved outside
            if (abs(idx.x()) >= num_cells.x()
                || abs(idx.y()) >= num_cells.y())
            {
                clear();
                return;
            }

            GridCell &src = *cells;

            GridCell tmp;
            tmp.resize(boost::extents[num_cells.x()][num_cells.y()]);
            std::fill(tmp.data(), tmp.data() + tmp.num_elements(), nullptr);

            boost::swap(tmp, src);

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
                        std::swap(src[x_new][y_new], tmp[x][y]);
                    }
                }
            }
            
            void **it = tmp.data();
            void **end = tmp.data() + num_cells.prod();
            
            //if elements are empty in new grid, swap and clear them
            for (unsigned int x = 0; x < num_cells.x(); ++x)
            {
                for (unsigned int y = 0; y < num_cells.y(); ++y)
                {
                    if(!src[x][y])
                    {
                        //search patch, that is not null;
                        while((*it) == nullptr && it != end)
                            it++;
                        
                        if(it == end)
                            throw std::runtime_error("Internal error, this should never happen");
                        
                        std::swap(src[x][y], *it);
                        factory->setDefault(src[x][y]);
                    }
                }
            }
        }
        
        T& get(Index idx)
        {
            return *(static_cast<T *> ( (*this->cells)[idx.x()][idx.y()]));
        }

        const T& get(Index idx) const
        {
            return *(static_cast<const T *> (*(this->cells)[idx.x()][idx.y()]));
        }

        Vector2ui getNumCells() const
        {
            return Vector2ui(cells->shape()[0], cells->shape()[1]);
        };
        
        void clear()
        {
            const Vector2ui num_cells(getNumCells());

            for (unsigned int x = 0; x < num_cells.x(); ++x)
            {
                for (unsigned int y = 0; y < num_cells.y(); ++y)
                {
                    factory->setDefault((*cells)[x][y]);
                }
            }
        };
    };

    
    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename T>
    class GridMap: public Grid
    {

    protected:
        /** Store the actual content of the cells of the grid **/
        GridStorage<T> *storage;

    public:

        template <class TBASE>
        GridMap<TBASE> *toBaseGrid()
        {
            static_assert(
                std::is_base_of<TBASE, T>::value, 
                "T must be a descendant of TBASE"
            );
            
            return new GridMap<TBASE>(*this, storage->template cast<TBASE>());
        }
        
        GridMap() 
            : Grid(),
             storage(new GridStorage<T>(Vector2ui(0,0), T()))

        {
//             static_assert(
//                 std::is_base_of<TBASE, T>::value, 
//                 "T must be a descendant of MyBase"
//             );
        }

        GridMap(const Grid &grid, GridStorage<T> *store)
            : Grid(grid),
              storage(store)
        {
        }
        
        GridMap(const GridMap& other)
            : Grid(other),
              storage(new GridStorage<T>(*other.storage))
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
              storage(new GridStorage<T>(num_cells, default_value))
        {
        }

        GridMap(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const T& default_value,
                const boost::shared_ptr<LocalMapData> &data)
            : Grid(num_cells, resolution, data),
              storage(new GridStorage<T>(num_cells, default_value))
        {}

        /** @brief default destructor
         */
        ~GridMap()
        {
            delete storage;
        }

    public:
        const T& at(const Vector3d& pos) const
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid.");
            return storage->get(idx);
        }

        T& at(const Vector3d& pos)
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid");
            return storage->get(idx);
        }

        const T& at(Index idx) const
        {
            if (!this->inGrid(idx))
                throw std::runtime_error("Provided index is out of the grid");
            return storage->get(idx);
        }

        T& at(Index idx)
        {
            if (!this->inGrid(idx))
                throw std::runtime_error("Provided index is out of the grid");
            return storage->get(idx);
        }

        const T& at(size_t x, size_t y) const
        {
            Index idx(x, y);
            if (!this->inGrid(idx))
                throw std::runtime_error("Provided index is out of the grid");
            return storage->get(idx);
        }

        T& at(size_t x, size_t y)
        {
            Index idx(x, y);
            if (!this->inGrid(idx))
                throw std::runtime_error("Provided index is out of the grid");
            return storage->get(idx);
        }
        
        const T& getDefaultValue() const
        {
            return storage->getDefaultValue();
        }

        template<class Q = T>
        void forEach(std::function<void (const Index &idx, const Q &elem)> f) const
        {
            Vector2ui numCells(storage->getNumCells());
            
            for(int x = 0; x < numCells.x(); x++)
            {
                for(int y = 0; y < numCells.y(); y++)
                {
                    Index cur(x,y);
                    f(cur, storage->get(cur));
                }
            }
        }

        
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
            Vector2ui numCells(storage->getNumCells());
            
            std::cout << "Num Cells is " << numCells.transpose() << std::endl;
            if(numCells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute max on empty map");

            Index maxIdx(0,0);
            Q max = storage->get(maxIdx);

            forEach<Q>([&](const Index &idx, const Q &elem){
                if(elem > max)
                {
                    maxIdx = idx;
                    max = elem;
                }
            }
            );
            
            return storage->get(maxIdx);
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
            Vector2ui numCells(storage->getNumCells());
            
            if(numCells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute min on empty map");

            Index minIdx(0,0);
            Q min = storage->get(minIdx);

            forEach<Q>([&](const Index &idx, const Q &elem){
                if(elem < min)
                {
                    minIdx = idx;
                    min = elem;
                }
            }
            );
            
            return storage->get(minIdx);
        }

        void moveBy(Index idx)
        {
            storage->moveBy(idx);
        }

        void clear()
        {
            storage->clear();
        }

    protected:
    };
    
}}
#endif // __ENVIRE_MAPS_GRID_MAP_HPP__
