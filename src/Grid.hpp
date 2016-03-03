#ifndef __ENVIRE_MAPS_GRID_HPP__
#define __ENVIRE_MAPS_GRID_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/function.hpp>

#include "LocalMap.hpp"

namespace envire { namespace maps 
{

    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;

    /**@brief type for grid resolution
     */
    typedef Eigen::Vector2d Vector2d;

    /**@brief type for grid position
     */
    typedef Eigen::Vector3d Vector3d;

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    class Index : public Vector2ui
    {
        public:
            typedef Vector2ui Base;

            Index()
                :  Vector2ui(0, 0)
            {}

            Index(unsigned int x, unsigned int y) 
                : Vector2ui(x, y)
            {}

            /* @brief this constructor allows you to construct Index from Eigen expressions
             */
            template<typename OtherDerived>
            Index(const Eigen::MatrixBase<OtherDerived>& other)
                : Vector2ui(other)
            {}

            /** @brief Lexicographical ordering (Strict Total Ordering)
             * @details
             * Returns true if:
             *      - index.x() < other.x()
             *      - index.x() == other.x() && index.y() < other.y()
             * Otherwise, return false.
             */
            bool operator<(const Index& other) const
            {
                return (x() < other.x() 
                        || (x() == other.x() && y() < other.y()));
            }

            /**
             * @brief Check if index is inside the other.
             * @details 
             * Returns true if x and y of index are smaller the x and y of other
             * @return [description]
             */
            bool isInside(const Index& other) const
            {
                return (x() < other.x() && y() < other.y());
            }
    };

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
             * Resolution in local X-axis and Y-axis
             * (Size of the cell in local X-axis and Y-axis in world unit)
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
    };
}}

#endif // __ENVIRE_MAPS_GRIDBASE_HPP__
