#pragma once
#include <vizkit3d/Vizkit3DPlugin.hpp>

/** Base types **/
#include <base/Eigen.hpp>

/** OSG **/
#include <osg/Geometry>

/** Boost **/
#include <boost/noncopyable.hpp>

/** The type to visualize **/
#include <maps/geometric/Point.hpp>
#include <maps/geometric/LineSegment.hpp>
#include <maps/geometric/ContourMap.hpp>

/** Std **/
#include <vector>

namespace vizkit3d
{

    class ContourMapVisualization
    : public vizkit3d::Vizkit3DPlugin<::maps::geometric::ContourMap>
    , boost::noncopyable
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
