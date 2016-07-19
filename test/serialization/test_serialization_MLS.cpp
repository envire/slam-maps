#define BOOST_TEST_MODULE SerializationTest
#include <boost/test/unit_test.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/** Based local map **/
#include <maps/grid/MLSMap.hpp>

using namespace ::maps::grid;

BOOST_AUTO_TEST_CASE(test_mls_surfacepatchbase_serialization)
{
    SurfacePatchBase sp_o(0.5, 1.3, 23);

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << sp_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    SurfacePatchBase sp_i;
    (*ia) >> sp_i;

    BOOST_CHECK(sp_i.getMin() == sp_o.getMin()); 
    BOOST_CHECK(sp_i.getMax() == sp_o.getMax());
    BOOST_CHECK(sp_i.getTop() == sp_o.getTop());
    BOOST_CHECK(sp_i.getBottom() == sp_o.getBottom());
    BOOST_CHECK(sp_i.isNegative() == sp_o.isNegative());
}


BOOST_AUTO_TEST_CASE(test_mls_surfacepatchslope_serialization)
{
    SurfacePatch<MLSConfig::SLOPE> sp_o(Eigen::Vector3f(-3.2, 2.5, -4.5), 4.6);

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << sp_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    SurfacePatch<MLSConfig::SLOPE> sp_i;
    (*ia) >> sp_i;

    BOOST_CHECK(sp_i.getMin() == sp_o.getMin()); 
    BOOST_CHECK(sp_i.getMax() == sp_o.getMax());
    BOOST_CHECK(sp_i.getTop() == sp_o.getTop());
    BOOST_CHECK(sp_i.getBottom() == sp_o.getBottom());
    BOOST_CHECK(sp_i.isNegative() == sp_o.isNegative());

    BOOST_CHECK(sp_i. getCenter() == sp_o. getCenter()); 
    BOOST_CHECK(sp_i.getNormal() == sp_o.getNormal());
}

BOOST_AUTO_TEST_CASE(test_mls_surfacepatchkalman_serialization)
{
    SurfacePatch<MLSConfig::KALMAN> sp_o(Eigen::Vector3f(-3.2, 2.5, -4.5), 4.6);

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << sp_o;

    // deserialize from string stream
    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    SurfacePatch<MLSConfig::KALMAN> sp_i;
    (*ia) >> sp_i;

    BOOST_CHECK(sp_i.getMin() == sp_o.getMin()); 
    BOOST_CHECK(sp_i.getMax() == sp_o.getMax());
    BOOST_CHECK(sp_i.getTop() == sp_o.getTop());
    BOOST_CHECK(sp_i.getBottom() == sp_o.getBottom());
    BOOST_CHECK(sp_i.isNegative() == sp_o.isNegative());

    BOOST_CHECK(sp_i. getCenter() == sp_o. getCenter()); 
    BOOST_CHECK(sp_i.getNormal() == sp_o.getNormal());
}

BOOST_AUTO_TEST_CASE(test_mls_serialization)
{
    //    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);
    Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //mls_config.updateModel = MLSConfig::KALMAN;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;


    for (unsigned int x = 0; x < numCells.x(); ++x) for(float dx = -.5f; dx <0.49f; dx+=0.125)
    {
        float xx = x+dx-numCells.x()/2;
        float cs = std::cos(xx * M_PI/50);
        for (unsigned int y = 0; y < numCells.y(); ++y) for (float dy = -0.5f; dy<0.49; dy+=0.125)
        {
            float yy = y+dy-numCells.y()/2;
            float sn = std::sin(yy* M_PI/50);

            mls_o.mergePoint(Eigen::Vector3d(xx*res.x(), yy*res.y(), cs*sn));
        }
    }

    std::stringstream stream;
    boost::archive::binary_oarchive oa(stream);
    oa << mls_o;  

    boost::archive::binary_iarchive *ia = new boost::archive::binary_iarchive(stream);
    MLSMapSloped mls_i;
    (*ia) >> mls_i;

    // Grid configuration
    //BOOST_CHECK(mls_o.getDefaultValue() == mls_i.getDefaultValue()); 
    BOOST_CHECK(mls_o.getResolution() == mls_i.getResolution()); 
    BOOST_CHECK(mls_o.getNumCells() == mls_i.getNumCells());

    // MLSGrid configuration
    BOOST_CHECK(mls_o.getConfig().gapSize == mls_i.getConfig().gapSize);
    BOOST_CHECK(mls_o.getConfig().thickness == mls_i.getConfig().thickness);
    BOOST_CHECK(mls_o.getConfig().useColor == mls_i.getConfig().useColor);
    BOOST_CHECK(mls_o.getConfig().updateModel == mls_i.getConfig().updateModel);
    BOOST_CHECK(mls_o.getConfig().useNegativeInformation == mls_i.getConfig().useNegativeInformation);

    for(size_t x = 0; x < numCells.x(); ++x)
    {
        for(size_t y = 0; y < numCells.y(); ++y)
        {
            Index idx(x,y);
            typedef MLSMapSloped::CellType Cell;
            const Cell &cell_o = mls_o.at(idx);
            const Cell &cell_i = mls_i.at(idx);

            // check the number of patches in the cells
            BOOST_CHECK_EQUAL(cell_o.size(), cell_i.size());

            // check the patches
            Cell::const_iterator it_o = cell_o.begin();
            Cell::const_iterator it_i = cell_i.begin();
            Cell::const_iterator end_o = cell_o.end();
            Cell::const_iterator end_i = cell_i.end();
            for(; it_o != end_o; ++it_o, ++it_i)
            {
                BOOST_CHECK(it_i != end_i);
                BOOST_CHECK_EQUAL(it_o->getMin(), it_i->getMin());
                BOOST_CHECK_EQUAL(it_o->getMax(), it_i->getMax());
                BOOST_CHECK_EQUAL(it_o->getTop(), it_i->getTop());
                BOOST_CHECK_EQUAL(it_o->getBottom(), it_i->getBottom());
                BOOST_CHECK_EQUAL(it_o->isNegative(), it_i->isNegative());
            }
            BOOST_CHECK(it_i == end_i);
        }
    }

}