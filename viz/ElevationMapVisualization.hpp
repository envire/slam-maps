#ifndef maps_ElevationGridVisualization_H
#define maps_ElevationGridVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include "ColorGradient.hpp"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/ElevationMap.hpp>

// TODO: should be replace by GridMapVisualization,
// since the ElevationMap is specification of GridMap

namespace vizkit3d
{
    class ElevationMapVisualization
        : public vizkit3d::Vizkit3DPlugin<::maps::grid::ElevationMap>
        , boost::noncopyable
    {
        Q_OBJECT

        public:
            ElevationMapVisualization();
            ~ElevationMapVisualization();

            Q_INVOKABLE void updateData(::maps::grid::ElevationMap const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::ElevationMap>::updateData(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::grid::ElevationMap const& plan);

        private:
            struct Data;
            Data* p;

            osg::HeightField* createHeighField();

            osg::Image* createTextureImage();

            ColorGradient heatMapGradient;

        public slots:
    };
}
#endif
