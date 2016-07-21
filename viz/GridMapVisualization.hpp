#ifndef maps_GridMapVisualization_H
#define maps_GridMapVisualization_H

#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/Vizkit3DPlugin.hpp>


#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/GridMap.hpp>

namespace vizkit3d
{
    class GridMapVisualization
        : public vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<float>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<int>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<char>>
        , boost::noncopyable
    {
        Q_OBJECT

        public:
            GridMapVisualization();
            ~GridMapVisualization();

            Q_PROPERTY(bool show_map_extents READ areMapExtentsShown WRITE setShowMapExtents)

            Q_INVOKABLE void updateData(::maps::grid::GridMap<double> const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}   
            Q_INVOKABLE void updateData(::maps::grid::GridMap<float> const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}    
            Q_INVOKABLE void updateData(::maps::grid::GridMap<int> const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}   
            Q_INVOKABLE void updateData(::maps::grid::GridMap<char> const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}                            

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::grid::GridMap<double> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<float> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<int> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<char> const& plan);            

            void setShowMapExtents(bool value);
            bool areMapExtentsShown() const;             

        private:
            struct Data;
            boost::scoped_ptr<Data> p;            

            bool showMapExtents;

        public slots:
    };
}
#endif