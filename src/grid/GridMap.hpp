#pragma once

/** std **/
#include <iostream>
#include <type_traits>

/** Boost **/
#include <boost/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

#include <maps/LocalMap.hpp>
#include <maps/grid/VectorGrid.hpp>

namespace maps { namespace grid
{
    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename CellT, typename GridT = VectorGrid<CellT> >
    class GridMap: public LocalMap, public GridT
    {

    private:
        /** 
         * @brief Resolution of the cell in local x-axis and y-axis
         * @details
         * Size of the cell in local x- und y-axis in some distance/length unit 
         * (e.g. meter or inch)
         * The unit used in the resolution should be the same as the unit used for
         * the translation part of the local frame. (s. LocalMapData::offset)
         */
        Vector2d resolution;

    public:
        typedef boost::shared_ptr<GridMap<CellT, GridT> > Ptr;
        typedef const boost::shared_ptr<GridMap<CellT, GridT> > ConstPtr;

        GridMap() 
            : LocalMap(maps::LocalMapType::GRID_MAP),
              GridT(),
              resolution(0,0)
        {
        }

        GridMap(const GridMap& other)
            : LocalMap(other), 
              GridT(other),
              resolution(other.resolution)
        {
        }

        template<class CellT2, class GridStorageT2>
        GridMap(const GridMap<CellT2, GridStorageT2>& other, const GridT& storage)
            : LocalMap(other)
            , GridT(storage)
            , resolution(other.getResolution())
        {
        }

        /** @brief Constructor of the abstract GridMap class
         *
         * Defines the extends and positioning of the grid. The grid is assumed
         * to be on the x-y plane of the reference frame. The number of grid
         * cells is given by the num_cells parameter. Each dimension (x and y)
         * also has an scaling and offset parameter, such that the origin of the
         * grid (grid_map) can be moved around and the grid scaled.
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
                const Vector2d &resolution,
                const CellT& default_value)
            : LocalMap(maps::LocalMapType::GRID_MAP),
              GridT(num_cells, default_value),
              resolution(resolution)
        {
        }

        GridMap(const Vector2ui &num_cells,
                const Vector2d &resolution,
                const CellT& default_value,
                const boost::shared_ptr<LocalMapData> &data)
            : LocalMap(data),
              GridT(num_cells, default_value),
              resolution(resolution)
        {}

        /** @brief default destructor
         */
        ~GridMap()
        {
        }

    public:
        using GridT::getNumCells;

        size_t getNumElements() const
        {
            return getNumCells().prod();
        }

        const Vector2d& getResolution() const 
        {
            return resolution;
        }

        void setResolution(const Vector2d& newRes)
        {
            if(newRes.isApprox(resolution, 0.00001))
                return;
            
            resolution = newRes;
            this->clear();
        }
        
        Vector2d getSize() const
        {
            return resolution.array() * getNumCells().template cast<double>().array();
        }

        bool inGrid(const Index& idx) const
        {
            // do not need to check idx against (0,0),
            // until idx is of type unsigned int
            return idx.isInside(getNumCells());
        }

        /** @brief get a position of an index from the Grid
         * 
         *  By default this function checks, if the index is within the grid,
         *  and returns false if the index is out of the grid.
         * 
         *  If the index is inside the grid, or checkIndex is set to false,
         *  this function will convert the given position into frame coordinates.
         * */
        bool fromGrid(const Index& idx, Vector3d& pos, bool checkIndex = true) const
        {
            /** Index inside the grid **/
            if (!checkIndex || (checkIndex && inGrid(idx)))
            {
                // position at the cell center without offset transformation
                Vector2d center = (idx.cast<double>() + Vector2d(0.5, 0.5)).array() * resolution.array();

                // Apply the offset transformation to the obtained position
                // pos_local = (Tgrid_local)^-1 * pos_grid
                pos = this->getLocalFrame().inverse() * Vector3d(center.x(), center.y(), 0.);
                return true;
            } else
            {
                /** Index outside the grid **/
                return false;
            }
        }

        /** @brief get a position of an index from the Grid applying an offset
         * set in the argument of the method
         * transformation. T_offset = frame_in_map = Tmap_frame
         * frame expressed with respect to the map_frame.
         * map_frame is the local_frame in case there is an offset or grid_map
         * in case there is not offset
         * */
        bool fromGrid(const Index& idx, Vector3d& pos_in_frame, const base::Transform3d &frame_in_map) const
        {
            Vector3d pos_in_map;

            if (fromGrid(idx, pos_in_map) == false)
                return false;

            /** Transform the position by the offset form the argument **/
            /** pos_in_frame = (Tmap_frame)^inverse * pos_in_map **/
            pos_in_frame = frame_in_map.inverse() * pos_in_map;

            return true;
        }

        /** @brief get the index to grid of a position
         * */
        bool toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_diff) const
        {
            if(toGrid(pos, idx))
            {
                Vector3d center;
                if(inGrid(idx) && fromGrid(idx, center))
                {
                    pos_diff = pos - center;
                    return true;
                }
            }
            return false;
        }

        /** @brief get an index of a position in the map applying an offset set
         * in the argument of the method.
         *
         * transformation. T_offset = frame_in_map = Tmap_frame
         * frame expressed with respect to the map_frame.
         * map_frame is the local_frame in case there is an offset or grid_map
         * in case there is not offset
         * */
        bool toGrid(const Vector3d& pos_in_frame, Index& idx, const base::Transform3d &frame_in_map) const
        {
            /** Transform the position by the offset form the argument **/
            /** pos_in_map = Tmap_frame * pos_in_frame **/
            Eigen::Vector3d pos_in_map = frame_in_map * pos_in_frame;

            return toGrid(pos_in_map, idx);
        }

        bool toGrid(const Vector3d& pos, Index& idx, bool checkIndex = true) const 
        {
            // Get the 2D position without the offset (in grid_frame)
            // pos_grid = Tgrid_local pos_local
            Vector2d pos_grid = Vector3d(this->getLocalFrame() * pos).head<2>();

            // Get the index for the pos_grid
            Eigen::Vector2d idx_double = pos_grid.array() / resolution.array();
            
            Index idx_temp(std::floor(idx_double.x()), std::floor(idx_double.y()));

            if(checkIndex && !inGrid(idx_temp))
            {
                return false;
            }
            
            idx = idx_temp;
            
            return true;
        }

        const CellT& at(const Vector3d& pos) const
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid.");
            return at(idx);
        }

        CellT& at(const Vector3d& pos)
        {
            Index idx;
            if (!this->toGrid(pos, idx))
                throw std::runtime_error("Provided position is out of the grid");
            return at(idx);
        }

        using GridT::at;

        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (integral and floating types)
         * hide this function e.g. for class types
         *
         * It also has the possibility to exclude the default_value
         *
         * @return [description]
         */
        template<class Q = CellT>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMax(const bool include_default_value = true) const
        {
            Vector2ui num_cells(getNumCells());

            std::cout << "Num Cells is " << num_cells.transpose() << std::endl;
            if(num_cells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute max on empty map");

            auto it = this->begin();
            auto endIt = this->end();

            const Q *first = &(*it);
//            const Q *last = &(*(this->end()));


            /** Include the default value as a possible max value to return **/
            if (include_default_value)
            {
                return *std::max_element(this->begin(), this->end());
            }
            else
            {
                const Q *largest = first;

                while (it != endIt)
                {
                    const Q *curElem = &(*it);

                    if(!this->isDefault(*largest))
                    {
                        if ((*largest < *curElem)&&(!this->isDefault(*curElem)))
                        {
                            largest = curElem;
                        }
                    }
                    else
                    {
                        if (!this->isDefault(*curElem))
                            largest = curElem;
                    }
                    it++;
                }
                return *largest;
            }
        }

        /**
         * @brief [brief description]
         * @details enable this function only for arithmetic types (integral and floating types)
         * hide this function e.g. for class types
         *
         * It also has the possibility to exclude the default_value
         *
         * @return [description]
         */
        template<class Q = CellT>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMin(const bool include_default_value = true) const
        {
            Vector2ui num_cells(getNumCells());

            std::cout << "Num Cells is " << num_cells.transpose() << std::endl;
            if(num_cells == Vector2ui(0,0))
                throw std::runtime_error("Tried to compute max on empty map");

            auto it = this->begin();
            auto endIt = this->end();

            const Q *first = &(*it);
//            const Q *last = &(*(this->end()));


            /** Include the default value as a possible max value to return **/
            if (include_default_value)
            {
                return *std::min_element(this->begin(), this->end());
            }
            else
            {
                const Q *smallest = first;

                while (it != endIt)
                {
                    const Q *curElem = &(*it);

                    if(!this->isDefault(*smallest))
                    {
                        if ((*curElem < *smallest) &&(!this->isDefault(*curElem)))
                        {
                            smallest = curElem;
                        }
                    }
                    else
                    {
                        if (!this->isDefault(*curElem))
                            smallest = curElem;
                    }
                    it++;
                }
                return *smallest;
            }
        }

        bool isDefault(const CellT &value) const
        {
            if (boost::math::isnan(this->getDefaultValue()))
            {
                return boost::math::isnan(value);
            }
            else
            {
                return value == this->getDefaultValue();
            }
        }

        void extend(const Vector2ui &minSize)
        {
            Vector2ui newSize = minSize.cwiseMax(getNumCells());
            this->resize(newSize);
            cell_extents.setEmpty();
        }

        CellExtents calculateCellExtents()
        {
            // set the cell_extents to maximal grid size
            if (cell_extents.isEmpty())
            {
                // set the cell_extents to initial size
                cell_extents.extend(Vector2ui(0, 0));
                cell_extents.extend(getNumCells());
            }

            Vector2ui num_cells = getNumCells();

            CellExtents new_cell_extents;
            bool has_value = false;

            // ------------ Left border
            // 1. check the current column of x_min and outside of it (range: 0 - x_min)
            int x_min = cell_extents.corner(CellExtents::BottomLeft).x();
            while (x_min >= 0)
            {
                // go through columns until at least one of the cells has value
                if (addCellForX(new_cell_extents, x_min) == true)
                    has_value = true;
                x_min--;
            }

            // 2. check inside columns, only if no values was found outside
            if (has_value == false)
            {
                x_min = cell_extents.corner(CellExtents::BottomLeft).x() + 1;
                while (x_min < num_cells.x() && has_value == false)
                {
                    // go through columns until at least one of the cells has value
                    if (addCellForX(new_cell_extents, x_min) == true)
                        has_value = true;
                    x_min++;                  
                }
            }

            // ------------- Right Border
            has_value = false;
            // 1. check the current column of x_min and outside of it (range: x_max - width)
            int x_max = cell_extents.corner(CellExtents::BottomRight).x();
            while (x_max < num_cells.x())
            {
                // go through columns until at least one of the cells has value
                if (addCellForX(new_cell_extents, x_max) == true)
                    has_value = true;
                x_max++;
            }            

            // 2. check inside columns, only if no values was found outside
            if (has_value == false)
            {
                x_max = cell_extents.corner(CellExtents::TopRight).x() - 1;
                while (x_max >= 0 && has_value == false)
                {
                    // go through columns until at least one of the cells has value
                    if (addCellForX(new_cell_extents, x_max) == true)
                        has_value = true;  
                    x_max--;                  
                }
            }         

            // ------------ Lower border   
            has_value = false;
            // 1. check the current column of x_min and outside of it (range: 0 - y_min)
            int y_min = cell_extents.corner(CellExtents::BottomLeft).y();
            while (y_min >= 0)
            {
                // go through columns until at least one of the cells has value
                if (addCellForY(new_cell_extents, y_min) == true)
                    has_value = true;
                y_min--;
            }

            // 2. check inside columns, only if no values was found outside
            if (has_value == false)
            {
                y_min = cell_extents.corner(CellExtents::BottomLeft).y() + 1;
                while (y_min < num_cells.y() && has_value == false)
                {
                    // go through columns until at least one of the cells has value
                    if (addCellForY(new_cell_extents, y_min) == true)
                        has_value = true;
                    y_min++;                  
                }
            }

            // ------------- Upper Border
            has_value = false;
            // 1. check the current column of x_min and outside of it (range: y_max - height)
            int y_max = cell_extents.corner(CellExtents::TopRight).y();
            while (y_max < num_cells.y())
            {
                // go through columns until at least one of the cells has value
                if (addCellForY(new_cell_extents, y_max) == true)
                    has_value = true;
                y_max++;
            }            

            // 2. check inside columns, only if no values was found outside
            if (has_value == false)
            {
                y_max = cell_extents.corner(CellExtents::TopRight).y() - 1;
                while (y_max >= 0 && has_value == false)
                {
                    // go through columns until at least one of the cells has value
                    if (addCellForY(new_cell_extents, y_max) == true)
                        has_value = true;
                    y_max--;                  
                }
            }                    

            cell_extents = new_cell_extents;
            return cell_extents;
        }

    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(::maps::LocalMap);
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridT);
            ar & BOOST_SERIALIZATION_NVP(resolution);
        }

    private:
        CellExtents cell_extents;

        bool addCellForX(CellExtents &new_cell_extents, int x)
        {
            unsigned int y = 0;
            while (y < getNumCells().y())
            {
                if (isDefault(at(x, y)) == false)
                {
                    new_cell_extents.extend(Vector2ui(x, y));                  
                    return true;
                }                      
                y++;
            }
            return false;
        }

        bool addCellForY(CellExtents &new_cell_extents, int y)
        {
            unsigned int x = 0;
            while (x < getNumCells().x())
            {
                if (isDefault(at(x, y)) == false)
                {
                    new_cell_extents.extend(Vector2ui(x, y));
                    return true;
                }                      
                x++;
            }
            return false;
        }
    };

    /** Typedef Grid Map types **/
    typedef GridMap<char> GridMapC; /* Char type of grid maps **/
    typedef GridMap<int> GridMapI;  /* Integer type of grid maps **/
    typedef GridMap<float> GridMapF; /* Float type of grid maps NOTE: use Elevation Map for this type **/
    typedef GridMap<double> GridMapD; /* double type of grid maps **/
}}