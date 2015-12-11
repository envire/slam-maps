#ifndef __ENVIRE_MAPS_GRID_HPP__
#define __ENVIRE_MAPS_GRID_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/intrusive_ptr.hpp>
#include <boost/function.hpp>

#include "LocalMap.hpp"

namespace envire { namespace maps 
{

    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;  

    typedef Eigen::Vector2d Vector2d;  

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    class Index : public Eigen::Matrix<unsigned int, 2, 1>
    {
        public:
            Index() 
                : Eigen::Matrix<unsigned int, 2, 1>(0, 0)
            {}

            Index(unsigned int x, unsigned int y) 
                : Eigen::Matrix<unsigned int, 2, 1>(x, y)
            {}

            bool operator<(const Index& other) const
            {
                return (x() < other.x() 
                    || (x() == other.x() && y() < other.y()));
            }           

            bool operator>(const Index& other) const
            {
                return (x() > other.x() 
                    || (x() == other.x() && y() > other.y()));
            }         

            // TODO: add exception if the other is bigger
            // than this. due to uint
            //Index operator-(const Index& other) const
            //{
            //}          
    };

    /** Base class for all 2D gridbased maps.
     * - desribes the grid structure: resolution and number of cells in x- and y- axes.
     * - converts positon -> index and index -> position
     * - checks if the position or index is inside the grid
     */
    class Grid : public LocalMap
    {
        public:
            typedef boost::intrusive_ptr<Grid> Ptr;

        private:
            /** Resolution in X-axis and Y-axis **/
            Vector2d resolution;

            /** Number of cells in X-axis **/
            Vector2ui num_cells;

        public:            
            Grid(const Vector2d &resolution, const Vector2ui &num_cells);

            ~Grid();
    };
}}

#endif // __ENVIRE_MAPS_GRIDBASE_HPP__