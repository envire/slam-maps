#define BOOST_TEST_MODULE VizTest
#include <boost/test/included/unit_test.hpp>
#include <boost/archive/polymorphic_binary_oarchive.hpp>

#include <Eigen/Geometry>

#include "StandaloneVisualizer.hpp"
#include <maps/grid/MLSMap.hpp>

using namespace ::maps::grid;

template<class MLSMap>
static void show_MLS(const MLSMap& mls)
{
    std::cout << "update finish" << std::endl;
    StandaloneVisualizer app;
    app.updateData(mls);

    while (app.wait(1000))
    {
    }
}

template<MLSConfig::update_model mlsType>
void mls_waves(const std::string& filename)
{
    //    GridConfig conf(300, 300, 0.05, 0.05, -7.5, -7.5);
    Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    typedef MLSMap<mlsType> MLSMap;
    mls_config.updateModel = mlsType;
    MLSMap *mls = new MLSMap(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls->getLocalFrame().translation() << 0.5*mls->getSize(), 0;

    /** Equivalent to translate the grid in opposite direction **/
    //Eigen::Vector3d offset_grid;
    //offset_grid << -0.5*mls->getSize(), 0.00;
    //mls->translate(offset_grid);

    for (unsigned int x = 0; x < numCells.x(); ++x) for(float dx = -.5f; dx <0.49f; dx+=0.125)
    {
        float xx = x+dx-numCells.x()/2;
        float cs = std::cos(xx * M_PI/50);
        for (unsigned int y = 0; y < numCells.y(); ++y) for (float dy = -0.5f; dy<0.49; dy+=0.125)
        {
            float yy = y+dy-numCells.y()/2;
            float sn = std::sin(yy* M_PI/50);

            mls->mergePoint(Eigen::Vector3d(xx*res.x(), yy*res.y(), cs*sn));
            //mls->at(x, y).update(SurfacePatch(Eigen::Vector3f(dx*res.x(),dy*res.y(),cs*sn), 0.1));
            //mls->at(x, y).update(SurfacePatch(cs*sn+10, 0.1, 9, SurfacePatch::NEGATIVE));
            //            mls->at(x, y).update(SurfacePatch(height, 0.1));
        }
    }

    if(!filename.empty())
    {
        std::ofstream of(filename.c_str(), std::ios::binary);
        boost::archive::binary_oarchive oa(of);
        oa << *mls;
    }

    show_MLS(*mls);

}

BOOST_AUTO_TEST_CASE( mlsviz_test )
{
    mls_waves<MLSConfig::SLOPE>("MLSMapSloped_waves.bin");
    mls_waves<MLSConfig::KALMAN>("MLSMapKalman_waves.bin");
}

BOOST_AUTO_TEST_CASE(mls_loop)
{
    //    GridConfig conf(150, 150, 0.1, 0.1, -7.5, -7.5);
    Eigen::Vector2d res(0.1, 0.1);
    Vector2ui numCells(150, 150);

    MLSConfig mls_config;
    mls_config.updateModel = MLSConfig::SLOPE;
    //    mls_config.updateModel = MLSConfig::KALMAN;
    mls_config.gapSize = 0.05f;
    mls_config.useNegativeInformation = false;
    float R = 5.0f, r=2.05f;
    MLSMapSloped *mls = new MLSMapSloped(numCells, res, mls_config);
    mls->getLocalFrame().translation() << 0.5*mls->getSize(), 0;

    for (float alpha = 0; alpha < M_PI; alpha += M_PI/1024/4)
    {
        float cs = std::cos(alpha);
        float sn = std::sin(alpha);
        for (float beta = 0; beta <= 2*M_PI; beta+=M_PI/256/4)
        {
            // Points of a torus:
            float x = (R+r*std::cos(beta)) * cs;
            float y = (R+r*std::cos(beta)) * sn;
            float z = r*std::sin(beta);

            mls->mergePoint(Vector3d(x,y,z));

        }
    }
    std::ofstream of("MLSMapSloped_loop.bin", std::ios::binary);
    boost::archive::binary_oarchive oa(of);
    oa << *mls;

    show_MLS(*mls);
}

