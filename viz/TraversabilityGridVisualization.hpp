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
#ifndef __MAPS_TRAVERSABILITY_GRID_VISUALIZATION_HPP_
#define __MAPS_TRAVERSABILITY_GRID_VISUALIZATION_HPP_

#include <vizkit3d/MapVisualization.hpp>

#include <osg/Geometry>
#include <osg/Image>
# include <osg/Texture2D>

#if QT_VERSION >= 0x050000 || !defined(Q_MOC_RUN)
    #include <maps/grid/TraversabilityGrid.hpp>
#endif

namespace vizkit3d
{
    /**
     * @brief: Visualization class for the traversabilityGrid.
     * @details: Offers some options to customize the visualiation at runtime:
     *  - interpolateCellColors: If true interpolates colors between cells.
     *  - useNPOTTextures:       If true enables the use of non-power-of-two textures.
     *                           (May be problematic on older systems that lack support for this.)
     *  - unknownColor:          Sets the color for grid cells with no/lacking information.
     *  - obstacleColor:         Sets the color for grid cells identified as obstacles.
     *                           (Set this to a different color than minTraversableColor to discern
     *                           obstacles form minimally traversable cells.)
     *  - minTraversableColor/maxTraversableColor:
     *                           The visualization will choose a color from this range depending on
     *                           the cells traversability.
     * NOTE: For a representation closest to the grid data set interpolateCellColors to false and
     *       useNPOTTextures to true.
     */
    class TraversabilityGridVisualization
        : public vizkit3d::MapVisualization<::maps::grid::TraversabilityGrid>
    {
        Q_OBJECT

        Q_PROPERTY(bool showMapExtents READ areMapExtentsShown WRITE setShowMapExtents)
        Q_PROPERTY(bool interpolateCellColors READ areCellColorsInterpolated WRITE setInterpolateCellColors)
        Q_PROPERTY(bool useNPOTTextures READ areNPOTTexturesUsed WRITE setUseNPOTTextures)
        Q_PROPERTY(QColor colorUnknown READ getUnknownColor WRITE setUnknownColor)
        Q_PROPERTY(QColor colorObstacle READ getObstacleColor WRITE setObstacleColor)
        Q_PROPERTY(QColor colorMinTraversable READ getMinTraversableColor WRITE setMinTraversableColor)
        Q_PROPERTY(QColor colorMaxTraversable READ getMaxTraversableColor WRITE setMaxTraversableColor)

    public:
        TraversabilityGridVisualization();
        ~TraversabilityGridVisualization();

        Q_INVOKABLE void updateData(const maps::grid::TraversabilityGrid& sample)
        {vizkit3d::Vizkit3DPlugin<maps::grid::TraversabilityGrid>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(::maps::grid::TraversabilityGrid const& plan);

        void visualize(osg::Geode& geode) const;
        // Creates a plane geometry as a basis for the visualization.
        osg::ref_ptr<osg::Geometry> createPlane();
        // Updates the plane to represent the current internal data.
        void updatePlane() const;
        // Creates an Image from the internal data to be applied to the
        // plane as a texture.
        void updateTextureImage() const;
        osg::Vec4 colorForCoordinate(std::size_t x, std::size_t y) const;

        // Node holding the plane.
        osg::ref_ptr<osg::Geode> planeGeode;
        osg::ref_ptr<osg::Geometry> plane;
        osg::ref_ptr<osg::Image> image;
        osg::ref_ptr<osg::Texture2D> texture;

    private:
        maps::grid::TraversabilityGrid traversabilityGrid;

        bool interpolateCellColors;
        bool useNPOTTextures;
        osg::Vec4 unknownColor;
        osg::Vec4 obstacleColor;
        osg::Vec4 minTraversableColor;
        osg::Vec4 maxTraversableColor;

    public slots:

        // Interpolation of colors between adjacent cells.
        bool areCellColorsInterpolated() const;
        // Interpolation of colors between adjacent cells.
        void setInterpolateCellColors(bool enabled);

        // Use non-power-of-two textures.
        bool areNPOTTexturesUsed() const;
        // Use non-power-of-two textures.
        void setUseNPOTTextures(bool enabled);

        // Color for UNKNOWN cells as RGBA (floats [0, 1]).
        QColor getUnknownColor() const;
        // Color for UNKNOWN cells as RGBA (floats [0, 1]).
        void setUnknownColor(QColor& color);

        // Color for OBSTACLE cells as RGBA (floats [0, 1]).
        QColor getObstacleColor() const;
        // Color for OBSTACLE cells as RGBA (floats [0, 1]).
        void setObstacleColor(QColor& color);

        // Color for the minimal traversability that is not an OBSTACLE as RGBA (floats [0, 1]).
        QColor getMinTraversableColor() const;
        // Color for the minimal traversability that is not an OBSTACLE as RGBA (floats [0, 1]).
        void setMinTraversableColor(QColor& color);

        // Color for maximal traversability as RGBA (floats [0, 1])
        QColor getMaxTraversableColor() const;
        // Color for maximal traversability as RGBA (floats [0, 1])
        void setMaxTraversableColor(QColor& color);
    };

}  // end namespace vizkit3d

#endif  // __MAPS_TRAVERSABILITY_GRID_VISUALIZATION_HPP_
