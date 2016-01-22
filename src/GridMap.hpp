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

    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename T>
    class GridMap: public Grid
    {

    public:
        typedef boost::multi_array<T, 2> ArrayType;

    private:
        /** default value type **/
        T default_value;

        /** Store the actual content of the cells of the grid **/
        ArrayType* cells;

    public:

        GridMap() 
            : Grid(), 
              cells(new ArrayType())
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
              default_value(default_value),
              cells(new ArrayType())
        {
        }

        GridMap(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const T& default_value,
                const boost::shared_ptr<LocalMapData> &data)
            : Grid(num_cells, resolution, data),
              default_value(default_value),
              cells(new ArrayType())
        {}

        GridMap(const GridMap& other)
            : Grid(other),
              default_value(other.default_value)
        {
            /** Get the cells **/
            this->cells = new ArrayType(*(other.cells));
        }

        /** @brief default destructor
         */
        ~GridMap()
        {
            delete cells;
        }

    public:
        /********************************************/
        /** GridMap method to operate with the map **/
        /********************************************/

        const T& getDefaultValue() const
        {
            return default_value;
        }

        const T& at(const Vector3d& pos) const
        {
            Index idx;
            if (!toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid.");
            return get(idx);
        }

        T& at(const Vector3d& pos)
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

        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (intergral and floating types)
         * hide this function e.g. for class types
         * @return [description]
         */
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMax() const
        {
            const ArrayType &array = getCells();
            return *(std::max_element(array.origin(), array.origin() + array.num_elements()));
        }

        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (intergral and floating types)
         * hide this function e.g. for class types
         * @return [description]
         */
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMin() const
        {
            const ArrayType &array = getCells();
            return *(std::min_element(array.origin(), array.origin() + array.num_elements()));
        }

        void moveBy(Index idx)
        {
            // if all grid values should be moved outside
            if (abs(idx.x()) >= this->num_cells.x()
                || abs(idx.y()) >= this->num_cells.y())
            {
                init();
                return;
            }

            ArrayType &src = getCells();

            ArrayType tmp;
            tmp.resize(boost::extents[this->num_cells.x()][this->num_cells.y()]);
            std::fill(tmp.data(), tmp.data() + tmp.num_elements(), default_value);

            boost::swap(tmp, src);

            for (unsigned int y = 0; y < this->num_cells.y(); ++y)
            {
                for (unsigned int x = 0; x < this->num_cells.x(); ++x)
                {
                    int x_new = x + idx.x();
                    int y_new = y + idx.y();

                    if ((x_new >= 0 && x_new < this->num_cells.x())
                        && (y_new >= 0 && y_new < this->num_cells.y()))
                    {
                        get(Index(x_new, y_new)) = *(tmp.data() + y * this->num_cells.x() + x);
                    }
                }
            }
        }

        void clear()
        {
            init();
        }

    private:
        T& get(Index idx)
        {
            return *(getCells().data() + idx.y() * this->num_cells.x() + idx.x());
        }

        const T& get(Index idx) const
        {
            return *(getCells().data() + idx.y() * this->num_cells.x() + idx.x());
        }

        void init() const
        {
            if (this->num_cells == Vector2ui::Zero())
                throw std::runtime_error("The grid size is zero! (therefore the array to hold the cells could be not allocated properly)");

            cells->resize(boost::extents[this->num_cells.x()][this->num_cells.y()]);
            std::fill(cells->data(), cells->data() + cells->num_elements(),
                    default_value);
        }

        ArrayType& getCells()
        {
            if (cells->num_elements() == 0)
                init();

            return *cells;
        }

        const ArrayType& getCells() const
        {
            if (cells->num_elements() == 0)
                init();

            return *cells;
        }
    };
}}
#endif // __ENVIRE_MAPS_GRID_MAP_HPP__
