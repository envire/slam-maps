#pragma once
#include <vizkit3d/Vizkit3DPlugin.hpp>

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
    : public vizkit3d::Vizkit3DPlugin<::maps::ContourMap>
    , boost::noncopyable
    {
        Q_OBJECT

    public:
        ContourMapVisualization();
        virtual ~ContourMapVisualization();

        Q_INVOKABLE void updateData(::maps::ContourMap const &sample)
        {
            vizkit3d::Vizkit3DPlugin<::maps::ContourMap>::updateData(sample);
        }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
	virtual void updateMainNode( osg::Node* node );
	virtual void updateDataIntern( const ::maps::ContourMap & data );

    private:
        /** Internal storage **/
        maps::ContourMap contour_lines;

        /** OSG drawable storage **/
        osg::ref_ptr<osg::Vec3Array> line_points;
        osg::ref_ptr<osg::DrawArrays> draw_arrays;
	osg::ref_ptr<osg::Geometry> geom;
	osg::ref_ptr<osg::Geode> geode;


    };

}
