#ifndef __ENVIRE_MAPS_GRID_MAP_HPP__
#define __ENVIRE_MAPS_GRID_MAP_HPP__

#include "LocalMap.hpp"

/** std **/
#include <iostream>

/** Eigen **/
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

/** Boost **/
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>

namespace envire {namespace maps
{

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    typedef Eigen::Vector2i Index;

    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;

    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename T>
    class GridMap: public LocalMap
    {

    public:
        typedef boost::multi_array<T, 2> ArrayType;

    private:

        /** Resolution in X-axis and Y-axis **/
        Eigen::Vector2d resolution;

        /** Number of cells in X-axis **/
        Vector2ui num_cells;

        /** default value type **/
        T default_value;

        /** Store the actual content of the cells of the grid **/
        ArrayType* cells;

    public:

        GridMap() :cells(new ArrayType())
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
        GridMap(
                const Eigen::Vector2d &_resolution,
                const Vector2ui &_num_cells,
                const T& _default_value):
                resolution(_resolution),
                num_cells(_num_cells),
                default_value(_default_value),
                cells(new ArrayType())
        {
        }

        GridMap(const GridMap& other):
            default_value(other.default_value)
        {
            /** Get the attribute values **/
            this->resolution = other.resolution;
            this->num_cells = other.num_cells;
            this->default_value = other.default_value;

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
        /******************************************/
        /** Base method to access the grid cells **/
        /******************************************/

        /** @brief get the number of cells
         */
        //Vector2ui &getNumCells const {return this->num_cells; }

        Eigen::Vector2d &getResolution() const { return this->resolution; }

        Eigen::Vector2d getSize() const { return this->num_cells * this->resolution; }

        /** @brief toGrid
         */
        bool toGrid(const Eigen::Vector2d& pos, Index& idx) const
        {
            Eigen::Vector2d pos_diff;
            return toGrid(pos.x(), pos.y(), idx.x(), idx.y(), pos_diff.x(), pos_diff.y());
        }

        /** @brief toGrid
         */
        bool toGrid(const Eigen::Vector2d& pos, Index &idx, Eigen::Vector2d& pos_diff) const
        {
            return toGrid(pos.x(), pos.y(), idx.x(), idx.y(), pos_diff.x(), pos_diff.y());
        }

        /** @brief toGrid
         */
        bool toGrid(const Eigen::Vector3d& pos_in_frame, Index& idx, const Eigen::Affine3d &frame_in_grid) const
        {
            Eigen::Vector3d pos_in_map = frame_in_grid * pos_in_frame;
            Eigen::Vector2d pos_diff;
            return toGrid(pos_in_map.x(), pos_in_map.y(), idx.x(), idx.y(), pos_diff.x(), pos_diff.y());
        }

        /** @brief fromGrid
         */
        bool fromGrid(const Index& idx, Eigen::Vector2d& pos) const
        {
            return fromGrid(idx.x(), idx.y(), pos.x(), pos.y());
        }

        /** @brief fromGrid
         */
        bool fromGrid(const Index& idx, Eigen::Vector3d& pos_in_frame, const Eigen::Affine3d &frame_in_grid) const
        {
            Eigen::Vector2d pos_in_map;
            bool result = fromGrid(idx.x(), idx.y(), pos_in_map.x(), pos_in_map.y());

            if (result == false)
                return false;

            pos_in_frame = frame_in_grid.inverse() * Eigen::Vector3d(pos_in_map.x(), pos_in_map.y(), 0.);

            return true;
        }

        /** @brief inGrid
         */
        bool inGrid(const Index& idx) const
        {
            return inGrid(idx.x, idx.y);
        }

    protected:

        /******************************************/
        /** Base method to access the grid cells **/
        /******************************************/

        /** @brief toGrid
         * Converts coordinates in the map-local frame to grid coordinates
         * @return true if (x, y) is within the grid and false otherwise
         */
        bool toGrid(double &x, double &y, size_t& xi, size_t& yi, double& xmod, double& ymod) const
        {
            /** TO-DO: this method should use vectors **/
            size_t xi_t = floor((x - offset.translation().x()) / resolution.x());
            size_t yi_t = floor((y - offset.translation().y()) / resolution.y());

            if(inGrid(xi_t, yi_t))
            {
                xi = xi_t;
                yi = yi_t;
                /** TO-DO: use the offset propertly as a transformation **/
                xmod = x - (xi * this->resolution.x() + this->offset.translation().x());
                ymod = y - (yi * this->resolution.y() + this->offset.translation().y());
                return true;
            }
            else
            {
                return false;
            }
        }

        /** @brief fromGrid
        * Converts coordinates from the map-local grid coordinates to
        * the coordinates in the specified \c frame
        */
        bool fromGrid(size_t xi, size_t yi, double& x, double& y) const
        {
            /** TO-DO: use vectors **/
            if (inGrid(xi, yi))
            {
                x = (xi + 0.5) * this->resolution.x() + this->offset.translation().x();
                y = (yi + 0.5) * this->resolution.y() + this->offset.translation().y();
                return true;
            }
            else
            {
                return false;
            }
        }

        /** @brief check whether the cell is in grid
         */
        bool inGrid(size_t xi, size_t yi) const
        {
            return (0 <= xi && xi < this->num_cells.x() && 0 <= yi && yi < this->num_cells.y());
        }

 public:
        /********************************************/
        /** GridMap method to operate with the map **/
        /********************************************/

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
            const ArrayType &array = getCells();
            return *(std::max_element(array.origin(), array.origin() + array.num_elements()));
        }

        const T& getMin() const
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

            for (int x = 0; x < this->num_cells.x(); ++x)
            {
                for (int y = 0; y < this->num_cells.y(); ++y)
                {
                    int x_new = x + idx.x();
                    int y_new = y + idx.y();

                    if ((x_new >= 0 && x_new < this->num_cells.x())
                        && (y_new >= 0 && y_new < this->num_cells.y()))
                    {
                        get(Index(x_new, y_new)) = *(tmp.data() + x * this->num_cells.x() + y);
                    }
                }
            }
        }

        void clear()
        {
            init();
        }
    };
}}
#endif // __ENVIRE_MAPS_GRID_MAP_HPP__
