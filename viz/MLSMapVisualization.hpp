#ifndef maps_MLSMapVisualization_H
#define maps_MLSMapVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>

#include <boost/noncopyable.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <maps/grid/MLSMap.hpp>

namespace vizkit3d
{
    class MLSMapVisualization
        : public vizkit3d::Vizkit3DPlugin<::maps::MLSMapKalman>
        , public vizkit3d::VizPluginAddType<::maps::MLSMapSloped>
        , boost::noncopyable
    {
        Q_OBJECT

        Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)
        Q_PROPERTY(bool show_negative READ isNegativeShown WRITE setShowNegative)
        Q_PROPERTY(bool show_extents READ areExtentsShown WRITE setShowExtents)        
        Q_PROPERTY(bool estimate_normals READ areNormalsEstimated WRITE setEstimateNormals)
        Q_PROPERTY(bool show_normals READ areNormalsShown WRITE setShowNormals)
        Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
        Q_PROPERTY(double cycle_color_interval READ getCycleColorInterval WRITE setCycleColorInterval)
        Q_PROPERTY(QColor horizontal_cell_color READ getHorizontalCellColor WRITE setHorizontalCellColor)
        Q_PROPERTY(QColor vertical_cell_color READ getVerticalCellColor WRITE setVerticalCellColor)
        Q_PROPERTY(QColor negative_cell_color READ getNegativeCellColor WRITE setNegativeCellColor)
        Q_PROPERTY(QColor uncertainty_color READ getUncertaintyColor WRITE setUncertaintyColor)        

        public:
            MLSMapVisualization();
            ~MLSMapVisualization();

            Q_INVOKABLE void updateData(::maps::MLSMapKalman const &sample)
            {updateDataIntern(sample);}
            Q_INVOKABLE void updateData(::maps::MLSMapSloped const &sample)
            {updateDataIntern(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(::maps::MLSMapSloped const& plan);
            virtual void updateDataIntern(::maps::MLSMapKalman const& plan);
            
        private:
            struct Data;
            boost::scoped_ptr<Data> p;
        
        public slots:

            bool isUncertaintyShown() const;
            void setShowUncertainty(bool enabled);

            bool isNegativeShown() const;
            void setShowNegative(bool enabled);

            bool areNormalsEstimated() const;
            void setEstimateNormals(bool enabled);

            bool areNormalsShown() const;
            void setShowNormals(bool enabled);

            bool isHeightColorCycled() const;
            void setCycleHeightColor(bool enabled);

            double getCycleColorInterval() const;
            void setCycleColorInterval(double interval);

            QColor getHorizontalCellColor() const;
            void setHorizontalCellColor(QColor color);

            QColor getVerticalCellColor() const;
            void setVerticalCellColor(QColor color);

            QColor getNegativeCellColor() const;
            void setNegativeCellColor(QColor color);

            QColor getUncertaintyColor() const;
            void setUncertaintyColor(QColor color);

            void setShowExtents( bool value );
            bool areExtentsShown() const;        

        protected:
            osg::Vec4 horizontalCellColor;
            osg::Vec4 verticalCellColor;
            osg::Vec4 negativeCellColor;
            osg::Vec4 uncertaintyColor;

            bool showUncertainty;
            bool showNegative;
            bool estimateNormals;
            bool showNormals;
            bool cycleHeightColor;
            double cycleColorInterval;
            bool showExtents;

#if 0
            osg::Vec3 estimateNormal(
                const ::maps::MLSMap &grid, 
                const ::maps::SurfacePatch &patch, 
                const ::maps::Index &patch_idx) const;
#endif
    };
}
#endif
