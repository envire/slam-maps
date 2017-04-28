//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
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

    protected:
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
        typedef CellT CellType;
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
            return fromGrid(idx, pos, 0.0, checkIndex);
        }

        /** @brief get a position of an index and a given height z inside the Grid
         *
         *  By default this function checks, if the index is within the grid,
         *  and returns false if the index is out of the grid.
         *
         *  If the index is inside the grid, or checkIndex is set to false,
         *  this function will convert the given position into frame coordinates.
         * */
        bool fromGrid(const Index& idx, Vector3d& pos, double z, bool checkIndex = true) const
        {
            /** Index inside the grid **/
            if (!checkIndex || inGrid(idx))
            {
                // position at the cell center without offset transformation
                Vector2d center = (idx.cast<double>() + Vector2d(0.5, 0.5)).array() * resolution.array();

                // Apply the offset transformation to the obtained position
                // pos_local = (Tgrid_local)^-1 * pos_grid
                pos = this->getLocalFrame().inverse(Eigen::Isometry) * Vector3d(center.x(), center.y(), z);
                return true;
            } else
            {
                /** Index outside the grid **/
                return false;
            }
        }


        /** @brief The inverse function to  toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_diff)
         * Computes the original position of a point, given its Index in the map, and it position in the cell.
         */
        bool fromGrid(const Index& idx, Vector3d& pos, const Vector3d& pos_in_cell, bool checkIndex = true) const
        {
            if(checkIndex && !inGrid(idx)) return false;
            // position at the cell center without offset transformation
            Vector3d pos_in_grid = pos_in_cell;
            pos_in_grid.head<2>() += (idx.cast<double>() + Vector2d(0.5,0.5)).cwiseProduct(resolution);
//            pos_in_grid.head<2>().array() *= resolution.array();

            // Apply the offset transformation to the obtained position
            // pos_local = (Tgrid_local)^-1 * pos_grid
            pos = this->getLocalFrame().inverse(Eigen::Isometry) * pos_in_grid;
            return true;
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
            pos_in_frame = frame_in_map.inverse(Eigen::Isometry) * pos_in_map;

            return true;
        }

        /** @brief get the index to grid of a position
         * */
        bool toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_in_cell) const
        {
            if(toGrid(pos, idx))
            {
                Vector3d center;
                if(inGrid(idx) && fromGrid(idx, center))
                {
                    pos_in_cell = this->getLocalFrame().rotation() * (pos - center);
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
         *
         * @note Prefer using \c prepareToGridOptimized and \c toGridOptimized
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
        /**
         * Returns a Transform3d object which transform from a local frame to the grid frame and scales to the grid resolution.
         * The return value of this shall be passed to \c toGridOptimized.
         */
        base::Transform3d prepareToGridOptimized(const base::Transform3d& frame2world)
        {
            base::Transform3d trafo = Eigen::DiagonalMatrix<double,3>(1.0/resolution.x(), 1.0/resolution.y(), 1.0) * this->getLocalFrame() * frame2world;
            trafo.translation().head<2>().array()-=0.5; // x and y coordinates shall be rounded down

            return trafo;

        }

        /** @brief optimized variant of toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_in_cell)
         */
        bool toGridOptimized(const Vector3d& pos, Index& idx, Vector3d& pos_in_cell, const base::Transform3d& trafo)
        {
            Vector3d pos_in_grid = trafo * pos;

            Index idx_temp(std::round(pos_in_grid.x()), std::round(pos_in_grid.y()));

            if(inGrid(idx_temp))
            {
                idx = idx_temp;
                pos_in_cell << (pos_in_grid.head<2>() - idx.cast<double>()).cwiseProduct(resolution), pos_in_grid.z();
                return true;
            }
            return false;
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
        }

        CellExtents calculateCellExtents() const
        {
            Vector2ui num_cells = getNumCells();

            CellExtents cell_extents;
            bool has_value = false;

            // ------------ Left border
            // x: from 0 until first value is found
            // y: from 0 to getNumCells().y()
            int x_min = -1;
            while (has_value == false && x_min < int(num_cells.x() - 1))
            {
                x_min++;
                // go through columns until at least one of the cells has value
                if (addCellForX(cell_extents, x_min, 0, num_cells.y() - 1) == true)
                {
                    has_value = true;
                }
            }

            // if no value was found until this point,
            // there is no data except default value in the grid
            if (has_value == false)
            {
                return cell_extents;
            }

            // ---------- Upper border
            // y: from 0 until first value is found
            // x : from x_min to getNumCells().x() 
            has_value = false;
            int y_min = -1;
            while (has_value == false)
            {
                y_min++;
                // go through columns until at least one of the cells has value
                if (addCellForY(cell_extents, y_min, x_min, num_cells.x() - 1) == true)
                {
                    has_value = true;
                }                
            }

            // ------------ Right border 
            // x: from getNumCells().x() - 1 until first value is found
            // y: from y_min until getNumCells().y()
            has_value = false;
            int x_max = num_cells.x();
            while (has_value == false)
            {
                x_max--;
                // go through columns until at least one of the cells has value
                if (addCellForX(cell_extents, x_max, y_min, num_cells.y() - 1) == true)
                {
                    has_value = true;
                }
            }

            // ---------- Bottom border
            // y: from getNumCells().y() - 1 until first value is found
            // x : from x_min to x_max
            has_value = false;
            int y_max = num_cells.y();
            while (has_value == false)
            {
                y_max--;
                // go through columns until at least one of the cells has value
                if (addCellForY(cell_extents, y_max, x_min, x_max) == true)
                {
                    has_value = true;
                }               
            }          

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
        bool addCellForX(CellExtents &cell_extents, unsigned int x, unsigned int y_start, unsigned int y_end) const
        {
            unsigned int y = y_start;
            while (y <= y_end)
            {
                if (isDefault(at(x, y)) == false)
                {
                    cell_extents.extend(Vector2ui(x, y));                  
                    return true;
                }                      
                y++;
            }
            return false;
        }

        bool addCellForY(CellExtents &cell_extents, unsigned int y, unsigned int x_start, unsigned int x_end) const
        {
            unsigned int x = x_start;
            while (x <= x_end)
            {
                if (isDefault(at(x, y)) == false)
                {
                    cell_extents.extend(Vector2ui(x, y));
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
