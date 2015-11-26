#ifndef __ENVIRE_MAPS_GRID_HPP__
#define __ENVIRE_MAPS_GRID_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/intrusive_ptr.hpp>
#include <boost/function.hpp>

#include "LocalMap.hpp"

namespace envire { namespace maps 
{
    /** Base class for all 2D gridbased maps.
     * - desribes the grid structure: resolution and number of cells in x- and y- axes.
     * - converts positon -> index and index -> position
     * - checks if the position or index is inside the grid
     */

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    typedef Eigen::Vector2i Index;

    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;

    class Grid : public LocalMap
    {
        public:
            typedef boost::intrusive_ptr<Grid> Ptr;

        private:
            /** Resolution in X-axis and Y-axis **/
            Eigen::Vector2d resolution;

            /** Number of cells in X-axis **/
            Vector2ui num_cells;

        public:            
            Grid(const Eigen::Vector2d &resolution, const Vector2ui &num_cells);

            ~Grid();
    };
}}

#endif // __ENVIRE_MAPS_GRIDBASE_HPP__