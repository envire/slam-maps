#pragma once

#include "Grid.hpp"
#include "storage/GridStorage.hpp"

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


namespace envire {namespace maps
{
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
        
        using R::getNumCells;

    protected:
                /** Grants access to boost serialization */
        friend class boost::serialization::access;  

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(envire::maps::Grid);
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(R);
        }   
    };    
}}

