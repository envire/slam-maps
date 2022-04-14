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

/** Base types **/
#include <base/Eigen.hpp>

/** OSG **/
#include <osg/Geometry>

/** Boost **/
#include <boost/noncopyable.hpp>

/** The type to visualize **/
#if QT_VERSION >= 0x050000 || !defined(Q_MOC_RUN)
    #include <maps/geometric/Point.hpp>
    #include <maps/geometric/LineSegment.hpp>
    #include <maps/geometric/ContourMap.hpp>
#endif

/** Std **/
#include <vector>

namespace vizkit3d
{

    class ContourMapVisualization
        : public vizkit3d::MapVisualization<::maps::geometric::ContourMap>
    {
        Q_OBJECT
        Q_PROPERTY(double LineWidth READ getLineWidth WRITE setLineWidth)
        Q_PROPERTY(QColor Color READ getColor WRITE setColor)

    public:
        ContourMapVisualization();
        virtual ~ContourMapVisualization();

        Q_INVOKABLE void updateData(::maps::geometric::ContourMap const &sample)
        {
            vizkit3d::Vizkit3DPlugin<::maps::geometric::ContourMap>::updateData(sample);
        }

        void setColor(const base::Vector3d& color);
	void setColor(double r, double g, double b, double a);

    public slots:
        double getLineWidth();
	void setLineWidth(double line_width);
        void setColor(QColor color);
        QColor getColor() const;

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode( osg::Node* node );
	virtual void updateDataIntern( const ::maps::geometric::ContourMap & data );

    private:
        /** Internal storage **/
        ::maps::geometric::ContourMap contour_lines;

        /** OSG drawable storage **/
        osg::ref_ptr<osg::Vec3Array> line_points;
        osg::ref_ptr<osg::DrawArrays> draw_arrays;
	osg::ref_ptr<osg::Geometry> geom;
	osg::ref_ptr<osg::Geode> geode;
        osg::ref_ptr<osg::Vec4Array> color_array;

        /** Property color attribute **/
        osg::Vec4 color;

        /** Property line width attribute **/
        double line_width;

    };
}
