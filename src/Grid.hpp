#ifndef __ENVIRE_MAPS_GRID_HPP__
#define __ENVIRE_MAPS_GRID_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/function.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "LocalMap.hpp"
#include "Index.hpp"

namespace envire { namespace maps 
{

    /** Base class for all 2D gridbased maps.
     * - describes the grid structure: resolution and number of cells in x- and y- axes.
     * - converts position -> index and index -> position
     * - checks if the position or index is inside the grid
     */
    class Grid : public LocalMap
    {
        public:
            typedef boost::shared_ptr<Grid> Ptr;

        private:
            /** Number of cells in X-axis and Y-axis **/
            Vector2ui num_cells;

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
            Grid();

            Grid(const Vector2ui &num_cells, const Vector2d &resolution);

            /**
             * @brief [brief description]
             * @details share the LocalMapData
             * 
             * @param num_cells [description]
             * @param resolution [description]
             * @param data [description]
             */
            Grid(const Vector2ui &num_cells, 
                const Vector2d &resolution, 
                const boost::shared_ptr<LocalMapData> &data);

            virtual ~Grid();

            /** @brief get the number of cells
             */
            virtual const Vector2ui& getNumCells() const;

            size_t numElements() const
            {
                return getNumCells().prod();
            }


            /** 
             * @brief get the resolution of the grid
             * @return cell size in local X-axis and Y-axis in world unit
             */
            const Vector2d& getResolution() const;

            /**
             * @brief get the size of the grid in world unit
             * @return size of the grid in local X-axis and Y-axis in world unit
             */
            Vector2d getSize() const;

            /**
             * @brief check whether the index argument is inside the grid
             * @return true or false
             */
            bool inGrid(const Index& idx) const;

            /**
             * @brief convert the index (the grid position) into position in local frame
             * by taking the offset (base::Transform3d) into account as description
             * from the local map
             *
             */
            bool fromGrid(const Index& idx, Vector3d& pos) const; 

            /**
             * @brief Converts coordinates from the map-local grid coordinates to
             * the coordinates in the specified \c frame
             */
            bool fromGrid(const Index& idx, Vector3d& pos_in_frame, const base::Transform3d &frame_in_grid) const;

            /**
             * @brief [brief description]
             * @details for the corner and border cases no definite solution 
             * 
             * @param pos position in the local frame
             * @param idx calculated index in teh grid
             * @param pos_diff difference between the position and the center of
             * the selected cell given by the calculated index. It gives an idea
             * of the estimated error.
             * 
             * @return [description]
             */
            bool toGrid(const Vector3d& pos, Index& idx) const;

            bool toGrid(const Vector3d& pos, Index& idx, Vector3d &pos_diff) const;

            bool toGrid(const Vector3d& pos_in_frame, Index& idx, const base::Transform3d &frame_in_grid) const;     

        private:
            /** Grants access to boost serialization */
            friend class boost::serialization::access;  

            /** Serializes the members of this class*/
            template <typename Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(envire::maps::LocalMap);
                ar & BOOST_SERIALIZATION_NVP(num_cells);
                ar & BOOST_SERIALIZATION_NVP(resolution);
            }    
                           
    };
}}

#endif // __ENVIRE_MAPS_GRIDBASE_HPP__
