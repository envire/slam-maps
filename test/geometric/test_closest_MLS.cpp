#define BOOST_TEST_MODULE SerializationTest
#include <boost/test/unit_test.hpp>


/** Based local map **/
#include "../common/GenerateMLS.hpp"

using namespace ::maps::grid;




BOOST_AUTO_TEST_CASE(test_mls_serialization)
{
    MLSMapSloped mls = generateWaves();



    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();
    double max_error = -1;
    for (double x = min.x(); x < max.x(); x += 0.00625*16)
    {
        std::cout << "\n\n\nx:" << x << '\n';
        double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625*16)
        {
            double sn = std::sin(y * M_PI/2.5);
            double z =-10;
            mls.getClosestSurfacePos(Eigen::Vector3d(x,y,z), z);
            max_error = std::max(std::abs(z-sn*cs), max_error );
            std::cout << "y: " << y << ", z: " << z << ", expected: " << sn*cs << '\n';
        }
    }
    std::cout << "\nMaxError: " << max_error << '\n';



}
