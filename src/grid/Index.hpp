#ifndef __MAPS_INDEX_HPP__
#define __MAPS_INDEX_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace maps { namespace grid
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    /**@brief type for the number of cells
     */
    typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;

    typedef Eigen::Vector2i Vector2i; 

    /**@brief type for grid resolution
     */
    typedef Eigen::Vector2d Vector2d;

    /**@brief type for grid position
     */
    typedef Eigen::Vector3d Vector3d;

    /**@brief Internal structure used to represent a position on the grid
     * itself, as a cell index
     */
    class Index : public Vector2i
    {
        public:
            typedef Vector2i Base;

            Index()
                :  Vector2i(0, 0)
            {}

            Index(int x, int y) 
                : Vector2i(x, y)
            {}

            /* @brief this constructor allows you to construct Index from Eigen expressions
             */
            template<typename OtherDerived>
            Index(const Eigen::MatrixBase<OtherDerived>& other)
                : Vector2i(other)
            {}

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

            /**
             * @brief Check if index is inside cell size.
             * @details 
             * Returns true if x and y of index are smaller the x and y of other
             * @return [description]
             */
            bool isInside(const Vector2ui& other) const
            {
                return (x() < static_cast<int>(other.x()) && y() < static_cast<int>(other.y())) && (x() >= 0.00 && y() >= 0.00);
            }

    };

    /** @brief Lexicographical ordering (Strict Total Ordering)
     * @details
     * Returns true if:
     *      - index.x() < other.x()
     *      - index.x() == other.x() && index.y() < other.y()
     * Otherwise, return false.
     */
    inline bool operator<(const Index& lhs, const Index& rhs)
    {
        return (lhs.x() < rhs.x()
                || (lhs.x() == rhs.x() && lhs.y() < rhs.y()));
    }

}}

#endif // __MAPS_INDEX_HPP__
