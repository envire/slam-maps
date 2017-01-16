#pragma once

#include <vizkit3d/Vizkit3DPlugin.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/OccupancyGridMap.hpp>

namespace vizkit3d
{

class OccupancyGridMapVisualization : public vizkit3d::Vizkit3DPlugin< ::maps::grid::OccupancyGridMap >
    , boost::noncopyable
{
    Q_OBJECT

    Q_PROPERTY(bool ShowOccupied READ getShowOccupied WRITE setShowOccupied)
    Q_PROPERTY(bool ShowFreespace READ getShowFreespace WRITE setShowFreespace)
    Q_PROPERTY(QColor OccupiedCellColor READ getOccupiedCellColor WRITE setOccupiedCellColor)
    Q_PROPERTY(QColor FreeSpaceCellColor READ getFreeSpaceCellColor WRITE setFreeSpaceCellColor)

    public:
        OccupancyGridMapVisualization();
        ~OccupancyGridMapVisualization();

        Q_INVOKABLE void updateData(::maps::grid::OccupancyGridMap const &sample)
        {vizkit3d::Vizkit3DPlugin<::maps::grid::OccupancyGridMap>::updateData(sample);}

    public slots:
        bool getShowOccupied() {return show_occupied;}
        void setShowOccupied(bool b) {show_occupied = b; emit propertyChanged("ShowOccupied"); setDirty();}
        bool getShowFreespace() {return show_freespace;}
        void setShowFreespace(bool b) {show_freespace = b; emit propertyChanged("ShowFreespace"); setDirty();}
        QColor getOccupiedCellColor() const {return occupied_color;}
        void setOccupiedCellColor(QColor color) {occupied_color = color; emit propertyChanged("OccupiedCellColor");}
        QColor getFreeSpaceCellColor() const {return free_space_color;}
        void setFreeSpaceCellColor(QColor color) {free_space_color = color; emit propertyChanged("FreeSpaceCellColor");}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(::maps::grid::OccupancyGridMap const& grid);

    private:
        maps::grid::OccupancyGridMap grid;
        bool show_occupied;
        bool show_freespace;
        QColor occupied_color;
        QColor free_space_color;
};

}