
/** Based local map **/
#include <maps/grid/MLSMap.hpp>


namespace maps {
namespace grid {






MLSMapSloped generateWaves()
{
    //    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);
    Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //mls_config.updateModel = MLSConfig::KALMAN;
    MLSMapSloped mls = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls.getLocalFrame().translation() << 0.5*mls.getSize(), 0;


    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();
    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double sn = std::sin(y * M_PI/2.5);
            mls.mergePoint(Eigen::Vector3d(x, y, cs*sn));
        }
    }
    return mls;
}

}  // namespace grid
}  // namespace maps
