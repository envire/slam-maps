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
        : public vizkit3d::Vizkit3DPlugin< ::maps::grid::MLSMapKalman >
        , public vizkit3d::VizPluginAddType< ::maps::grid::MLSMap<::maps::grid::MLSConfig::SLOPE> >
        , public vizkit3d::VizPluginAddType< ::maps::grid::MLSMap<::maps::grid::MLSConfig::PRECALCULATED> >
        , boost::noncopyable
    {
        Q_OBJECT

        Q_PROPERTY(bool show_map_extents READ areMapExtentsShown WRITE setShowMapExtents)
        Q_PROPERTY(bool show_uncertainty READ isUncertaintyShown WRITE setShowUncertainty)
        Q_PROPERTY(bool show_negative READ isNegativeShown WRITE setShowNegative)
        Q_PROPERTY(bool show_patch_extents READ arePatchExtentsShown WRITE setShowPatchExtents)        
        Q_PROPERTY(bool estimate_normals READ areNormalsEstimated WRITE setEstimateNormals)
        Q_PROPERTY(bool show_normals READ areNormalsShown WRITE setShowNormals)
        Q_PROPERTY(bool cycle_height_color READ isHeightColorCycled WRITE setCycleHeightColor)
        Q_PROPERTY(double cycle_color_interval READ getCycleColorInterval WRITE setCycleColorInterval)
        Q_PROPERTY(double uncertainty_scale READ getUncertaintyScale WRITE setUncertaintyScale)
        Q_PROPERTY(QColor horizontal_cell_color READ getHorizontalCellColor WRITE setHorizontalCellColor)
        Q_PROPERTY(QColor vertical_cell_color READ getVerticalCellColor WRITE setVerticalCellColor)
        Q_PROPERTY(QColor negative_cell_color READ getNegativeCellColor WRITE setNegativeCellColor)
        Q_PROPERTY(QColor uncertainty_color READ getUncertaintyColor WRITE setUncertaintyColor)

        public:
            MLSMapVisualization();
            ~MLSMapVisualization();

            Q_INVOKABLE void updateMLSPrecalculated(maps::grid::MLSMapPrecalculated const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateMLSSloped(maps::grid::MLSMapSloped const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateMLSKalman(maps::grid::MLSMapKalman const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapPrecalculated const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapSloped const &sample)
            {vizkit3d::Vizkit3DPlugin<::maps::grid::MLSMapKalman>::updateData(sample);}

            Q_INVOKABLE void updateData(maps::grid::MLSMapKalman const &sample)
            {vizkit3d::Vizkit3DPlugin<maps::grid::MLSMapKalman>::updateData(sample);}


        protected:
            virtual osg::ref_ptr<osg::Node> createMainNode();
            virtual void updateMainNode(osg::Node* node);
            virtual void updateDataIntern(maps::grid::MLSMapKalman const& mls);
            virtual void updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::SLOPE> const& mls);
            virtual void updateDataIntern(::maps::grid::MLSMap<::maps::grid::MLSConfig::PRECALCULATED> const& mls);
            
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

            void setShowPatchExtents( bool value );
            bool arePatchExtentsShown() const; 

            void setShowMapExtents(bool value);
            bool areMapExtentsShown() const;

            double getUncertaintyScale() const;
            void setUncertaintyScale(double scaling);

        protected:
            osg::Vec4 horizontalCellColor;
            osg::Vec4 verticalCellColor;
            osg::Vec4 negativeCellColor;
            osg::Vec4 uncertaintyColor;

            bool showMapExtents;
            bool showUncertainty;
            bool showNegative;
            bool estimateNormals;
            bool showNormals;
            bool cycleHeightColor;
            double cycleColorInterval;
            bool showPatchExtents;
            double uncertaintyScale;

#if 0
            osg::Vec3 estimateNormal(
                const ::maps::MLSMap &grid, 
                const ::maps::SurfacePatch &patch, 
                const ::maps::Index &patch_idx) const;
#endif
    };
}
#endif
