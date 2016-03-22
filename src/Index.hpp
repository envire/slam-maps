#ifndef __ENVIRE_MAPS_INDEX_HPP__
#define __ENVIRE_MAPS_INDEX_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace envire { namespace maps {
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

#endif // __ENVIRE_MAPS_INDEX_HPP__
