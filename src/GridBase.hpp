#ifndef __ENVIRE_MAPS_GRIDBASE_HPP__
#define __ENVIRE_MAPS_GRIDBASE_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/intrusive_ptr.hpp>
#include <boost/function.hpp>

#include "GridConfig.hpp"

namespace envire
{
    namespace maps 
    {
        /** Base class for all maps that function as regular grids
         *
         * This map offers a common interface for all maps that are regular grids
         *
         * The create() static method can be used as a factory method for all
         * subclasses that are registered on the serialization system
         *
         * A generic implementation as a template is Grid<T>, declared in Grid.hpp.
         * Specialized implementation for common types of maps can be found in
         * Grids.hpp (such as: ElevationMap, OccupancyGrid, ...)
         */
        class GridBase
        {
            public:
                typedef boost::intrusive_ptr<GridBase> Ptr;

                /** Internal structure used to represent a position on the grid itself,
                 * as a cell index
                 */
                // TODO: probably replace the Index with Vector2i or extend it
                struct Index
                {
                    /** The cell position along the X axis */
                    size_t x;
                    /** The cell position along the Y axis */
                    size_t y;

                    Index() : x(0), y(0) {}
                    Index( size_t x, size_t y ) : x(x), y(y) {}
                    explicit Index( const Eigen::Vector2i &index ) : x(index.x()), y(index.y()) {}

                    bool operator<( const Index& other ) const
                    {
                        if( x < other.x )
                            return true;
                        else
                        {
                            if( x == other.x )
                                return y < other.y;
                            else
                                return false;
                        }
                    }

                    bool operator==( const Index& other ) const
                    {
                        return x == other.x && y == other.y;
                    }

                    bool operator!=( const Index& other ) const
                    {
                        return x != other.x || y != other.y;
                    }

                    Index operator+(const Index& second) const
                    {
                        return Index(x + second.x, y + second.y);
                    }

                    // TODO: what should we do if the second index are bigger than the first one?
                    Index operator-(const Index& second) const
                    {
                        return Index(x - second.x, y - second.y);
                    }
                };          

                explicit GridBase();

                /** @brief Constructor of the abstract GridBase class
                 * 
                 * Defines the extends and positioning of the grid. The grid is assumed
                 * to be on the x-y plane of the reference frame. The number of grid
                 * cells is given by the cellSizeX and cellSizeY params. Each dimension
                 * also has an scaling and offset parameter, such that the origin of the
                 * grid can be moved around and the grid scaled.
                 *
                 * The relation between the grid cell index xi and the value for the
                 * dimension x is:
                 * @verbatim
                 * x = xi * scale_x + offset_x
                 * @endverbatim
                 * This is of course the same for the y axis as well.
                 *  
                 * @param cellSizeX - number of cells in x direction
                 * @param cellSizeY - number of cells in y direction
                 * @param scalex - scaling of the x axis (size in x per cell)
                 * @param scaley - scaling of the y axis (size in y per cell)
                 * @param offsetx - x-position of the [0,0] cell
                 * @param offsety - y-position of the [0,0] cell
                 */
                GridBase(GridConfig config);            

                virtual ~GridBase();

                const GridConfig& getGridConfig() const 
                {
                    return config;
                }
       
                /** Returns the size of the grid, in cells, along the X direction
                */
                size_t getCellSizeX() const { return config.cellSizeX; }

                /** Returns the size of the grid, in cells, along the Y direction
                 */
                size_t getCellSizeY() const { return config.cellSizeY; }

                /** Returns the world size of the grid along the X direction
                 */
                double getSizeX() const { return config.cellSizeX * config.scaleX; }

                /** Returns the world size of the grid along the Y direction
                 */
                double getSizeY() const { return config.cellSizeY * config.scaleY; }

                /** Returns the world size of a cell along the X direction
                 */
                double getScaleX() const { return config.scaleX; };
                /** Returns the world size of a cell along the Y direction
                 */
                double getScaleY() const { return config.scaleY; }

                /** Returns the X part of the position of the (0, 0) cell w.r.t. the
                 * grid's frame
                 */
                double getOffsetX() const { return config.offsetX; }

                /** Returns the Y part of the position of the (0, 0) cell w.r.t. the
                 * grid's frame
                 */
                double getOffsetY() const { return config.offsetY; }

                /** Converts coordinates in the map-local frame to grid coordinates
                 *
                 * @return true if (x, y) is within the grid and false otherwise
                 */
                bool toGrid(const Eigen::Vector2d& pos, Index& idx) const;      

                bool toGrid(const Eigen::Vector2d& pos, Index &idx, Eigen::Vector2d& posDiff) const;    

                bool toGrid(const Eigen::Vector3d& pos_in_frame, Index& idx, const Eigen::Affine3d &frame_in_grid) const;

                bool fromGrid(const Index& idx, Eigen::Vector2d& pos) const;

                /** Converts coordinates from the map-local grid coordinates to
                * the coordinates in the specified \c frame
                */
                bool fromGrid(const Index& idx, Eigen::Vector3d& pos_in_frame, const Eigen::Affine3d &frame_in_grid) const;

                bool inGrid(const Index& idx) const;

            protected:
                GridConfig config;

                /** Converts coordinates in the map-local frame to grid coordinates
                 * and calculate the position within the cell, such that xmod and ymod
                 * are between 0 and scalex or scaley respectively
                 *
                 * @return true if (x, y) is within the grid and false otherwise
                 */
                bool toGrid(double x, double y, size_t& xi, size_t& yi, double& xmod, double& ymod) const;  

                /** Converts coordinates from the map-local grid coordinates to
                 * coordinates in the map-local frame
                 * TODO: check if the indexes are inside the grid size
                 */
                bool fromGrid(size_t xi, size_t yi, double& x, double& y) const;    

                bool inGrid(size_t xi, size_t yi) const;                            

        };
    }
}

#endif // __ENVIRE_MAPS_GRIDBASE_HPP__