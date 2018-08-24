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
#ifndef maps_GridMapVisualization_H
#define maps_GridMapVisualization_H

#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

#include <vizkit3d/MapVisualization.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/GridMap.hpp>

namespace vizkit3d
{
    class GridMapVisualization
        : public vizkit3d::MapVisualization<::maps::grid::GridMap<double>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<float>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<int>>
        , public vizkit3d::VizPluginAddType<::maps::grid::GridMap<char>>
    {
        Q_OBJECT

        Q_PROPERTY(bool showMapExtents READ areMapExtentsShown WRITE setShowMapExtents)

        public:
            GridMapVisualization();
            ~GridMapVisualization();

            Q_INVOKABLE void updateGridMapD(::maps::grid::GridMapD const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}   
            Q_INVOKABLE void updateGridMapF(::maps::grid::GridMapF const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}    
            Q_INVOKABLE void updateGridMapI(::maps::grid::GridMapI const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}   
            Q_INVOKABLE void updateGridMapC(::maps::grid::GridMapC const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::GridMap<double>>::updateData(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::grid::GridMap<double> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<float> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<int> const& plan);
            virtual void updateDataIntern(::maps::grid::GridMap<char> const& plan);            

        private:
            struct Data;
            boost::scoped_ptr<Data> p;            

            osg::ref_ptr<osg::Geode> geode;


        public slots:
    };
}
#endif
