#include <boost/test/unit_test.hpp>
#include <envire_maps/SurfacePatch.hpp>

/**
 * Test cases for SurfacePatches.
 * Note: The SurfacePatch class may get separated into different classes for different update models.
 * Currently this only tests the SLOPE update model
 */


using namespace envire::maps;

typedef Eigen::Vector3f Vec3;
typedef Eigen::Vector2f Vec2;

BOOST_AUTO_TEST_CASE(test_surface_patch_basics)
{
    Vec2 zero2(0.0,0.0);
    SurfacePatch pt(Vec3(0.1,0.2,3), 1.0), pt2(Vec3(0.2,0.2,3), 1.0f), pt3(Vec3(0.2,0.1,3), 1.0f);
    BOOST_CHECK_EQUAL(pt.getWeight(), 1.0);
    BOOST_CHECK_EQUAL(pt.getMean(), 3.0f);
    BOOST_CHECK_EQUAL(pt.getHeight(), 0.0f);
    BOOST_CHECK_EQUAL(pt.getHeight(zero2), 3.0f); // WARNING: This only works as long as (x,y) of the point have magnitude < 1
    BOOST_CHECK(pt.getNormal().isApprox(Vec3::UnitZ(), 1e-5));
    bool merged = pt.mergePlane(pt2, MLSConfig());
    BOOST_CHECK(merged);
    BOOST_CHECK_EQUAL(pt.getWeight(), 2.0f);
    BOOST_CHECK_EQUAL(pt.getMean(), 3.0f);
    BOOST_CHECK_EQUAL(pt.getHeight(), 0.0f);
    BOOST_CHECK_EQUAL(pt.getHeight(zero2), 3.0f); // WARNING: This usually works only as long as (x,y) of both points have magnitude < 1
    BOOST_CHECK(pt.getNormal().isApprox(Vec3::UnitZ(), 1e-5));
    merged = pt.mergePlane(pt3, MLSConfig());
    BOOST_CHECK(merged);
    BOOST_CHECK_EQUAL(pt.getWeight(), 3.0f);
    BOOST_CHECK_EQUAL(pt.getMean(), 3.0f);
    BOOST_CHECK_EQUAL(pt.getHeight(), 0.0f);
    BOOST_CHECK_CLOSE_FRACTION(pt.getHeight(zero2), 3.0f, 1e-5f);
    BOOST_CHECK(pt.getNormal().isApprox(Vec3::UnitZ(), 1e-5));
}


BOOST_AUTO_TEST_CASE(test_surface_patch_normals)
{
    MLSConfig config; config.updateModel = MLSConfig::SLOPE;
    config.gapSize = 2.0f;

    for(int i=0; i<100; ++i)
    {
        // Generate a random normal vector pointing mostly upwards:
        Vec3 normal; normal << Vec2::Random(), 2.0f;
        normal.normalize();
        float d = rand() * 100.f/RAND_MAX;
        SurfacePatch pat(Vec3(0,0,d/normal.z()), 1.0f);
        for(int j=0; j<100; ++j)
        {
            Vec2 r = Vec2::Random();
            Vec3 x; x << r, (d-r.dot(normal.head<2>()))/normal.z();
            SurfacePatch p(x, 1.0f);
            BOOST_CHECK(pat.merge(p, config));
        }
        // Even though all points should be perfectly on a plane this gets quite high
        // TODO Try to improve accuracies then reduce test tolerances
        BOOST_CHECK_SMALL(pat.getStdev(), 0.1f);
        BOOST_CHECK(pat.getNormal().isApprox(normal, 1e-4));
        BOOST_CHECK_CLOSE(std::cos(pat.getSlope()), double(normal.z()), 5e-3);
    }
}

BOOST_AUTO_TEST_CASE(test_surface_patch_dont_merge)
{
    MLSConfig config; config.updateModel = MLSConfig::SLOPE;
    config.gapSize = 2.0;
    // pt1 and pt2 shall be patches getting the same points as input only with an offset of 10 units in z
    SurfacePatch pt1(Vec3::Zero(), 1.0), pt2(Vec3::UnitZ()*10, 1.0);
    for(int i=0; i<100; ++i)
    {
        Vec3 point = Vec3::Random();
        SurfacePatch ptA(point, 1.0);
        SurfacePatch ptB(point + 10.0 * Vec3::UnitZ(), 1.0);
        BOOST_CHECK(pt1.merge(ptA, config));
        BOOST_CHECK(pt2.merge(ptB, config));
        BOOST_CHECK(!pt1.merge(ptB, config));
        BOOST_CHECK(!pt2.merge(ptA, config));
        BOOST_CHECK(!pt1.merge(pt2, config));
    }

    BOOST_CHECK_SMALL(pt1.getMean(), 1.0f);
    BOOST_CHECK_CLOSE(pt2.getMean(), 10.0f, 1.0f);
    BOOST_CHECK(pt1.getNormal().isApprox(pt2.getNormal()));
    BOOST_CHECK_CLOSE(pt1.getSlope(), pt2.getSlope(), 1e-3);
    for(int i=0; i<50; ++i)
    {
        Eigen::Vector2f pos; pos.setRandom();
        BOOST_CHECK_CLOSE(pt1.getHeight(pos) + 10.0f, pt2.getHeight(pos), 1e-4);
    }

}
