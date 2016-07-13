#ifndef maps_GridMapVisualization_H
#define maps_GridMapVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

#include "ColorGradient.hpp"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/GridMap.hpp>

namespace vizkit3d
{
    class GridMapVisualization
        : public vizkit3d::Vizkit3DPlugin<::maps::grid::GridMapD>
        , boost::noncopyable
    {
        Q_OBJECT

        public:
            GridMapVisualization();
            ~GridMapVisualization();

            Q_PROPERTY(bool show_map_extents READ areMapExtentsShown WRITE setShowMapExtents)

            Q_INVOKABLE void updateData(::maps::grid::GridMapD const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMapD>::updateData(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::grid::GridMapD const& plan);

            void setShowMapExtents(bool value);
            bool areMapExtentsShown() const;             

        private:
            struct Data;
            Data* p;

            osg::HeightField* createHeighField();

            osg::Image* createTextureImage();

            ColorGradient heatMapGradient;

            bool showMapExtents;

        public slots:
    };
}
#endif