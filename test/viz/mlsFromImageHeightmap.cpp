#include <QImage>
#include "maps/grid/MLSConfig.hpp"
#include "maps/grid/MLSMap.hpp"
#include <boost/archive/polymorphic_binary_oarchive.hpp>
#include <base/Eigen.hpp>
#include "StandaloneVisualizer.hpp"
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

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

int main(int argc, char *argv[])
{
    
    if(argc <= 3)
    {
        std::cout << "Usage: heightmap_to_mls <input height map> <output file name> <max height>\n input = any grayscale image" << std::endl;
        exit(-1);
    }
    
    const double maxHeight = boost::lexical_cast<double>(argv[3]);
    QImage img(argv[1]);
    
    const int width = img.width();
    const int height = img.height();
    std::cout << "Image: width: " << width << ", height: " << height << std::endl;
    
    const Eigen::Vector2d res(0.1, 0.1);
    const Vector2ui numCells(width, height);

    MLSConfig mls_config;
//     mls_config.updateModel = MLSConfig::SLOPE;
    mls_config.updateModel = MLSConfig::KALMAN;
    mls_config.gapSize = 0.3f;
    mls_config.useNegativeInformation = false;
    MLSMapKalman mls(numCells, res, mls_config);
//     
    for(int x = 0; x < width; ++x)
    {
        for(int y = 0; y < height; ++y)
        {
            const QRgb pixel = img.pixel(x, y);
            const int brightness = qGray(pixel);
            const double topHeight = maxHeight * brightness / 255.0;
            
            for(double height = topHeight; height > -0.3; height -= 0.2)
            {
                base::Vector3d point(res.x() / 2.0 +  x * res.x(), res.y() / 2.0 + y * res.y(), height);
                mls.mergePoint(point);
            }
        }
    }

    std::ofstream of(argv[2], std::ios::binary);
    boost::archive::binary_oarchive oa(of);
    oa << mls;
    of.flush();
    
    show_MLS(mls);
        

    
    return 0;
}
