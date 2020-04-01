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

#include <vizkit3d/TraversabilityGridVisualization.hpp>
#include <vizkit3d/ExtentsRectangle.hpp>
#include <vizkit3d/ColorConversionHelper.hpp>


using namespace vizkit3d;

TraversabilityGridVisualization::TraversabilityGridVisualization()
    : interpolateCellColors(true),
      useNPOTTextures(false),
      unknownColor(osg::Vec4(0, 0, 1, 1)),
      obstacleColor(osg::Vec4(0.8, 0, 0, 1)),
      minTraversableColor(osg::Vec4(1, 0, 0, 1)),
      maxTraversableColor(osg::Vec4(0, 1, 0, 1))
{
}

TraversabilityGridVisualization::~TraversabilityGridVisualization()
{
}

osg::ref_ptr<osg::Node> TraversabilityGridVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();

    planeGeode = new osg::Geode();

    plane = createPlane();
    image = new osg::Image();
    texture = new osg::Texture2D(image.get());
    osg::ref_ptr<osg::StateSet> state = planeGeode->getOrCreateStateSet();

    state->setTextureAttributeAndModes(0, texture.get());

    planeGeode->addDrawable(plane.get());

    mainNode->addChild(planeGeode.get());

    return mainNode;
}

void TraversabilityGridVisualization::updateMainNode(osg::Node* node)
{
    // Draw map extents.
    visualizeMapExtents(traversabilityGrid.calculateCellExtents(), traversabilityGrid.getResolution());

    // Apply localFrame.
    setLocalFrame(traversabilityGrid.getLocalFrame());

    visualize(*planeGeode);
}

void TraversabilityGridVisualization::updateDataIntern(const maps::grid::TraversabilityGrid& plan)
{
    traversabilityGrid = plan;
}

void vizkit3d::TraversabilityGridVisualization::visualize(osg::Geode& geode) const
{
        updatePlane();

        updateTextureImage();

        texture->setResizeNonPowerOfTwoHint(!useNPOTTextures);

        if (!interpolateCellColors)
        {
            texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        }
        else
        {
            texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
            texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        }
}

osg::ref_ptr<osg::Geometry> TraversabilityGridVisualization::createPlane()
{
        osg::ref_ptr<osg::Geometry> plane = new osg::Geometry();

        // Assign white as a color to the plane.
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
        plane->setColorArray(c.get());
        plane->setColorBinding(osg::Geometry::BIND_OVERALL);
        c->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

        // Set texture coordinates.
        osg::ref_ptr<osg::Vec2dArray>  texture_coordinates = new osg::Vec2dArray;
        texture_coordinates->push_back(osg::Vec2d(0,0));
        texture_coordinates->push_back(osg::Vec2d(1,0));
        texture_coordinates->push_back(osg::Vec2d(1,1));
        texture_coordinates->push_back(osg::Vec2d(0,1));
        plane->setTexCoordArray(0, texture_coordinates.get());

        osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, 4);
        plane->addPrimitiveSet(drawArrays.get());

        return plane;
}

void TraversabilityGridVisualization::updatePlane() const
{
    maps::grid::Vector2d mapSize = traversabilityGrid.getSize();

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

    vertices->push_back(osg::Vec3(0, 0, 0));
    vertices->push_back(osg::Vec3(mapSize.x(), 0, 0));
    vertices->push_back(osg::Vec3(mapSize.x(), mapSize.y(), 0));
    vertices->push_back(osg::Vec3(0, mapSize.y(), 0));

    plane->setVertexArray(vertices.get());
}


void TraversabilityGridVisualization::updateTextureImage() const
{
        maps::grid::Vector2ui numCells = traversabilityGrid.getNumCells();

        unsigned char* imageData = new unsigned char[numCells.x() * numCells.y() * 4];
        unsigned char* pos = imageData;

        for (size_t y = 0; y < numCells.y(); ++y)
        {
            for (size_t x = 0; x < numCells.x(); ++x)
            {
                osg::Vec4 color = colorForCoordinate(x, y);

                *pos++ = static_cast<unsigned char>(color.r() * 255.0);
                *pos++ = static_cast<unsigned char>(color.g() * 255.0);
                *pos++ = static_cast<unsigned char>(color.b() * 255.0);
                *pos++ = static_cast<unsigned char>(color.a() * 255.0);
            }
        }

        image->setImage(numCells.x(),
                        numCells.y(),
                        1,  // datadepth per channel
                        GL_RGBA,
                        GL_RGBA,
                        GL_UNSIGNED_BYTE,  // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
                        imageData,  // unsigned char* data
                        osg::Image::USE_NEW_DELETE,  // USE_NEW_DELETE, //osg::Image::NO_DELETE,// AllocationMode mode (shallow copy)
                        1);
}

osg::Vec4 TraversabilityGridVisualization::colorForCoordinate(std::size_t x, std::size_t y) const
{
    float probability = traversabilityGrid.getProbability(x, y);
    if (probability < 0.001)
    {
        // unknown
        return unknownColor;
    }

    uint8_t traversabilityClassId = traversabilityGrid.getTraversabilityClassId(x, y);
    switch(traversabilityClassId)
    {
        case 0:
            // unknown
            return unknownColor;
        case 1:
            // obstacle
            return obstacleColor;
        default:
            // Interpolate between minTraversableColor and maxTraversableColor depending on drivability.
            float drivability = traversabilityGrid.getTraversability(x, y).getDrivability();

            float r = (1 - drivability) * minTraversableColor.r() + drivability * maxTraversableColor.r();
            float g = (1 - drivability) * minTraversableColor.g() + drivability * maxTraversableColor.g();
            float b = (1 - drivability) * minTraversableColor.b() + drivability * maxTraversableColor.b();
            float alpha = (1 - drivability) * minTraversableColor.a() + drivability * maxTraversableColor.a();
            return osg::Vec4(r, g, b, alpha);
    }
}

bool TraversabilityGridVisualization::areCellColorsInterpolated() const
{
    return interpolateCellColors;
}

void TraversabilityGridVisualization::setInterpolateCellColors(bool enabled)
{
    interpolateCellColors = enabled;
    emit propertyChanged("interpolateCellColors");
    setDirty();
}

bool TraversabilityGridVisualization::areNPOTTexturesUsed() const
{
    return useNPOTTextures;
}

void TraversabilityGridVisualization::setUseNPOTTextures(bool enabled)
{
    useNPOTTextures = enabled;
    emit propertyChanged("useNPOTTextures");
    setDirty();
}

QColor TraversabilityGridVisualization::getUnknownColor() const
{
    QColor color;
    color.setRgbF(unknownColor.r(), unknownColor.g(), unknownColor.b(), unknownColor.a());
    return color;
}

void TraversabilityGridVisualization::setUnknownColor(QColor& color)
{
    unknownColor.set(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("unknownColor");
    setDirty();
}

QColor TraversabilityGridVisualization::getObstacleColor() const
{
    QColor color;
    color.setRgbF(obstacleColor.r(), obstacleColor.g(), obstacleColor.b(), obstacleColor.a());
    return color;
}

void TraversabilityGridVisualization::setObstacleColor(QColor& color)
{
    obstacleColor.set(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("obstacleColor");
    setDirty();
}

QColor TraversabilityGridVisualization::getMinTraversableColor() const
{
    QColor color;
    color.setRgbF(minTraversableColor.r(), minTraversableColor.g(), minTraversableColor.b(), minTraversableColor.a());
    return color;
}

void TraversabilityGridVisualization::setMinTraversableColor(QColor& color)
{
    minTraversableColor.set(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("minTraversableColor");
    setDirty();
}

QColor TraversabilityGridVisualization::getMaxTraversableColor() const
{
    QColor color;
    color.setRgbF(maxTraversableColor.r(), maxTraversableColor.g(), maxTraversableColor.b(), maxTraversableColor.a());
    return color;
}

void TraversabilityGridVisualization::setMaxTraversableColor(QColor& color)
{
    maxTraversableColor.set(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("maxTraversableColor");
    setDirty();
}
