//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#pragma once

#include <vizkit3d/MapVisualization.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#if QT_VERSION >= 0x050000 || !defined(Q_MOC_RUN)
    #include <maps/grid/OccupancyGridMap.hpp>
#endif

namespace vizkit3d
{

class OccupancyGridMapVisualization : public vizkit3d::MapVisualization< ::maps::grid::OccupancyGridMap >
{
    Q_OBJECT

    Q_PROPERTY(bool showMapExtents READ areMapExtentsShown WRITE setShowMapExtents)
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
        osg::ref_ptr<osg::Group> localNode;

        maps::grid::OccupancyGridMap grid;
        bool show_occupied;
        bool show_freespace;
        QColor occupied_color;
        QColor free_space_color;
};

}
