#ifndef maps_ElevationGridVisualization_H
#define maps_ElevationGridVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include "ColorGradient.hpp"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/ElevationMap.hpp>

namespace vizkit3d
{
    class ElevationMapVisualization
        : public vizkit3d::Vizkit3DPlugin<::maps::ElevationMap>
        , boost::noncopyable
    {
        Q_OBJECT

        Q_PROPERTY(QStringList elevation_layer READ getElevationLayers WRITE setElevationLayer)

        public:
            ElevationMapVisualization();
            ~ElevationMapVisualization();

            Q_INVOKABLE void updateData(::maps::ElevationMap const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::ElevationMap>::updateData(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::ElevationMap const& plan);

        private:
            struct Data;
            Data* p;

            QString current_grid_key;
            QStringList grid_keys;

            osg::HeightField* createHeighField();

            osg::Image* createTextureImage();

            ::maps::ColorGradient heatMapGradient;

        public slots:
            QStringList getElevationLayers();
            QString getElevationLayer() const;
            void setElevationLayer(const QStringList &keys); 
            void setElevationLayer(const QString &key); 
    };
}
#endif
