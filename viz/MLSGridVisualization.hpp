#ifndef maps_MLSGridVisualization_H
#define maps_MLSGridVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>

#include <boost/noncopyable.hpp>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/Texture2D>

#include <envire_maps/MLSGrid.hpp>

namespace vizkit3d
{
    class MLSGridVisualization
        : public vizkit3d::Vizkit3DPlugin<envire::maps::MLSGrid>
        , boost::noncopyable
    {
        Q_OBJECT

        Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)
        Q_PROPERTY(bool show_negative READ isNegativeShown WRITE setShowNegative)
        Q_PROPERTY(bool show_extents READ areExtentsShown WRITE setShowExtents)        
        Q_PROPERTY(bool estimate_normals READ areNormalsEstimated WRITE setEstimateNormals)
        Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
        Q_PROPERTY(double cycle_color_interval READ getCycleColorInterval WRITE setCycleColorInterval)
        Q_PROPERTY(QColor horizontal_cell_color READ getHorizontalCellColor WRITE setHorizontalCellColor)
        Q_PROPERTY(QColor vertical_cell_color READ getVerticalCellColor WRITE setVerticalCellColor)
        Q_PROPERTY(QColor negative_cell_color READ getNegativeCellColor WRITE setNegativeCellColor)
        Q_PROPERTY(QColor uncertainty_color READ getUncertaintyColor WRITE setUncertaintyColor)        

        public:
            MLSGridVisualization();
            ~MLSGridVisualization();

            Q_INVOKABLE void updateData(envire::maps::MLSGrid const &sample)
            {vizkit3d::Vizkit3DPlugin<envire::maps::MLSGrid>::updateData(sample);}

        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(envire::maps::MLSGrid const& plan);
            
        private:
            struct Data;
            Data* p;
        
        public slots:

            bool isUncertaintyShown() const;
            void setShowUncertainty(bool enabled);

            bool isNegativeShown() const;
            void setShowNegative(bool enabled);

            bool areNormalsEstimated() const;
            void setEstimateNormals(bool enabled);

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
            bool cycleHeightColor;
            double cycleColorInterval;
            bool showExtents;

            osg::Vec3 estimateNormal(
                const envire::maps::MLSGrid &grid, 
                const envire::maps::SurfacePatch &patch, 
                const envire::maps::GridBase::Index &patch_idx) const;
    };
}
#endif
