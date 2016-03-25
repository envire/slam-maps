#ifndef __MAPS_GRIDMAP_HPP_
#define __MAPS_GRIDMAP_HPP_

#pragma once

#include <maps/LocalMap.hpp>
#include <maps/grid/GridCell.hpp>

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
#include <boost/math/special_functions/fpclassify.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>


namespace maps
{
    /**@brief GridMap class IEEE 1873 standard
     * This map is a Grid structure for a raster metric (Cartesian) map
     * This map offers a template class for all maps that are regular grids
     */
    template <typename T, typename R = GridCell<T> >
    class GridMap: public LocalMap, public R
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
        GridMap() 
            : LocalMap(maps::LocalMapType::GRID_MAP),
              R(),
              resolution(0,0)
        {
        }

        GridMap(const GridMap& other)
            : LocalMap(other), 
              R(other),
              resolution(other.resolution)
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
                const Eigen::Vector2d &resolution,
                const T& default_value)
            : LocalMap(maps::LocalMapType::GRID_MAP),
              R(num_cells, default_value),
              resolution(resolution)
        {
        }

        GridMap(const Vector2ui &num_cells,
                const Eigen::Vector2d &resolution,
                const T& default_value,
                const boost::shared_ptr<LocalMapData> &data)
            : LocalMap(data),
              R(num_cells, default_value),
              resolution(resolution)
        {}

        /** @brief default destructor
         */
        ~GridMap()
        {
        }

    public:
        using R::getNumCells;

        size_t getNumElements() const
        {
            return getNumCells().prod();
        }

        const Vector2d& getResolution() const 
        {
            return resolution;
        }

        Vector2d getSize() const
        {
            return resolution.array() * Vector2ui(getNumCells()).cast<double>().array();
        }

        bool inGrid(const Index& idx) const
        {
            // do not need to check idx against (0,0),
            // until idx is of type unsigned int
            return idx.isInside(Index(getNumCells()));
        }

        /** @brief get a position of an index from the Grid
         * */
        bool fromGrid(const Index& idx, Vector3d& pos) const
        {
            /** Index inside the grid **/
            if (inGrid(idx))
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
            // Get the 2D position without the offset (in grid_frame)
            // pos_grid = Tgrid_local pos_local
            Vector2d pos_grid = Vector3d(this->getLocalFrame() * pos).head<2>();

            // Get the index for the pos_grid
            // cast to float due to position which lies on the border between two cells
            Eigen::Vector2d idx_double = pos_grid.array() / resolution.array();
            Index idx_temp(std::floor(idx_double.x()), std::floor(idx_double.y()));

            Vector3d center;
            if(inGrid(idx_temp) && fromGrid(idx_temp, center))
            {
                idx = idx_temp;
                pos_diff = pos - center;
                return true;
            }
            else
            {
                return false;
            }
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

            Eigen::Vector3d pos_diff;
            return toGrid(pos_in_map, idx, pos_diff);
        }

        bool toGrid(const Vector3d& pos, Index& idx) const 
        {
            Vector3d pos_diff;
            return toGrid(pos, idx, pos_diff);
        }


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
         *
         * It also has the possibility to exclude the default_value
         *
         * @return [description]
         */
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMax(const bool include_default_value = true) const
        {
            Vector2ui numCells(getNumCells());

            std::cout << "Num Cells is " << numCells.transpose() << std::endl;
            if(numCells == Vector2ui(0,0))
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
        template<class Q = T>
        const typename std::enable_if<std::is_arithmetic<Q>::value, Q>::type&
        getMin(const bool include_default_value = true) const
        {
            Vector2ui numCells(getNumCells());

            std::cout << "Num Cells is " << numCells.transpose() << std::endl;
            if(numCells == Vector2ui(0,0))
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

        bool isDefault(const T &value) const
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

    protected:
                /** Grants access to boost serialization */
        friend class boost::serialization::access;  

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(::maps::LocalMap);
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(R);
            ar & BOOST_SERIALIZATION_NVP(resolution);
        }
    };
}

#endif /* __MAPS_GRIDMAP_HPP_ */
