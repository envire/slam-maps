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
#include <iostream>

#include "GridMapVisualization.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/TexMat>

#include "ColorGradient.hpp"

using namespace vizkit3d;
using namespace ::maps::grid;

struct GridMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    virtual ~Data() { }
    virtual void visualize(osg::Geode& geode, bool showHeightField, bool interpolateCellColors, bool useNPOTTextures) const = 0;
    virtual maps::grid::CellExtents getCellExtents() const = 0;
    virtual maps::grid::Vector2d getResolution() const = 0;
    virtual base::Affine3d getLocalFrame() const = 0;
};

template<typename GridT>
struct DataHold : public GridMapVisualization::Data
{
    GridMap<GridT> grid;

    ColorGradient heatMapGradient;

    DataHold(const GridMap<GridT> grid) 
        : grid(grid) 
    {
        this->heatMapGradient.createDefaultHeatMapGradient();
    }

    void visualize(osg::Geode& geode, bool showHeightField ,bool interpolateCellColors, bool useNPOTTextures) const
    {
        while(geode.removeDrawables(0));

        // Create height field.
        if (showHeightField)
        {
            osg::ref_ptr<osg::HeightField> heightField = createHeighField();

            // Add height field to geode.
            osg::ShapeDrawable *drawable = new osg::ShapeDrawable(heightField);
            geode.addDrawable(drawable);
        }
        else
        {
            osg::ref_ptr<osg::Geometry> plane = createPlane();
            geode.addDrawable(plane.get());
        }

        // Set material properties.
        osg::StateSet* state = geode.getOrCreateStateSet();
        osg::ref_ptr<osg::Material> mat = new osg::Material;
        mat->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );

        mat->setAmbient( osg::Material::FRONT_AND_BACK,
                osg::Vec4( .5f, .5f, .3f, 1.0f ) );
        mat->setDiffuse( osg::Material::FRONT_AND_BACK,
                osg::Vec4( .5f, .5f, .3f, 1.0f ) );
        //mat->setSpecular( osg::Material::FRONT,
        //          osg::Vec4( 1.f, 1.f, 1.f, 1.0f ) );

        state->setAttribute( mat.get() );

        osg::ref_ptr<osg::Image> image = createTextureImage();   
        osg::Texture2D* tex = new osg::Texture2D(image);

        if (interpolateCellColors)
        {
            tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
            tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        }
        else
        {
            tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST_MIPMAP_NEAREST);
            tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        }

        tex->setResizeNonPowerOfTwoHint(!useNPOTTextures);
        state->setTextureAttributeAndModes(0, tex);          

        if (showHeightField)
        {
            // Use osg::TexMat to set the scale and positioning of the texture.
            // NOTE: Necissary since the HeightField is drawn from cell center to cell center,
            //       thus being smaller than the texture is supposed to be.
            osg::Matrixd scaleMatrix, translationMatrix, scaleTranslationMatrix;
            float xScale = (grid.getNumCells().x() - 1) / static_cast<float>(grid.getNumCells().x());
            float yScale = (grid.getNumCells().y() - 1) / static_cast<float>(grid.getNumCells().y());
            float xTranslation = (1. - xScale) / 2;
            float yTranslation = (1. - yScale) / 2;

            scaleMatrix.makeScale(osg::Vec3(xScale, yScale, 0.));
            translationMatrix.makeTranslate(osg::Vec3(xTranslation, yTranslation, 0.));
            scaleTranslationMatrix = scaleMatrix * translationMatrix;

            osg::ref_ptr<osg::TexMat> textureMatrix = new osg::TexMat(scaleTranslationMatrix);

            state->setTextureAttributeAndModes(0, textureMatrix.get(), osg::StateAttribute::ON);
        }
        else
        {
            // Set texture without scaling when using the plane.
            osg::ref_ptr<osg::TexMat> textureMatrix = static_cast<osg::TexMat*>(state->getTextureAttribute(0, osg::StateAttribute::TEXMAT));
            if (textureMatrix)
            {
                textureMatrix->setMatrix(osg::Matrixd());
            }

        }
    };
    
    maps::grid::CellExtents getCellExtents() const
    {
        return grid.calculateCellExtents();
    }

    maps::grid::Vector2d getResolution() const
    {
        return grid.getResolution();
    }

    base::Affine3d getLocalFrame() const
    {
        return grid.getLocalFrame();
    }

    osg::ref_ptr<osg::Geometry> createPlane() const
    {
        maps::grid::Vector2d mapSize = grid.getSize();

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

        // Specify the vertices.
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        vertices->push_back(osg::Vec3(0, 0, 0));
        vertices->push_back(osg::Vec3(mapSize.x(), 0, 0));
        vertices->push_back(osg::Vec3(mapSize.x(), mapSize.y(), 0));
        vertices->push_back(osg::Vec3(0, mapSize.y(), 0));
        plane->setVertexArray(vertices.get());

        osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, vertices->size());
        plane->addPrimitiveSet(drawArrays.get());

        return plane;
    }

    osg::HeightField* createHeighField() const
    {
        // create height field
        osg::HeightField* heightField = new osg::HeightField();
        heightField->allocate(grid.getNumCells().x(), grid.getNumCells().y());
        heightField->setXInterval(grid.getResolution().x());
        heightField->setYInterval(grid.getResolution().y());
        heightField->setSkirtHeight(0.0f); 

        heightField->setOrigin(osg::Vec3(grid.getResolution().x() / 2.0, grid.getResolution().y() / 2, 0.0));

        double min = grid.getMin(false);
        double default_value = grid.getDefaultValue();

        for (unsigned int r = 0; r < heightField->getNumRows(); r++) 
        {
            for (unsigned int c = 0; c < heightField->getNumColumns(); c++) 
            {
                GridT cell_value = grid.at(maps::grid::Index(c, r));

                if( cell_value !=  default_value)
                    heightField->setHeight(c, r, cell_value);
                else
                    heightField->setHeight(c, r, min);    // min elevation
            }
        }     

        return heightField;  
    }    

    osg::Image* createTextureImage() const
    {
        osg::Image* image = new osg::Image(); 

        //convert double to uint16 
        int size = grid.getNumCells().x() * grid.getNumCells().y() * 4;
        unsigned char* image_raw_data = new unsigned char[size];
        unsigned char* pos = image_raw_data;

        GridT min = grid.getMin(false);
        GridT max = grid.getMax(false);

        //scaling between SCALING_MIN_VALUE and SCALING_MAX_VALUE meters 
        double scaling = std::abs(max - min);

        if(scaling == 0)
            scaling = 1.0;

        // fill image with color
        for (unsigned int y = 0; y < grid.getNumCells().y(); ++y)
        {
            for (unsigned int x = 0; x < grid.getNumCells().x(); ++x)
            {
                /** Get the cell value **/
                GridT cell_value = grid.at(maps::grid::Index(x, y));

                double normalize_value = (double)(cell_value - min)/scaling;
                osg::Vec4f col(1.0,1.0,0.6,1.0);
                this->heatMapGradient.getColorAtValue(normalize_value, col.r(),col.g(),col.b());

                *pos++ = (unsigned char)(col.r() * 255.0);
                *pos++ = (unsigned char)(col.g() * 255.0);
                *pos++ = (unsigned char)(col.b() * 255.0);
                *pos++ = (unsigned char)(col.a() * 255.0);
            }
        }

        image->setImage(
                grid.getNumCells().x(),
                grid.getNumCells().y(),
                1, // datadepth per channel
                GL_RGBA, 
                GL_RGBA, 
                GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
                (unsigned char*)(image_raw_data), // unsigned char* data
                osg::Image::USE_NEW_DELETE, // USE_NEW_DELETE, //osg::Image::NO_DELETE,// AllocationMode mode (shallow copy)
                1);      

        return image;
    }    
};

GridMapVisualization::GridMapVisualization()
    : MapVisualization<maps::grid::GridMap<double>>()
    , p(0)
    , showHeightField(false)
    , interpolateCellColors(true)
{}

GridMapVisualization::~GridMapVisualization()
{
}

osg::ref_ptr<osg::Node> GridMapVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = MapVisualization::createMainNode()->asGroup();
    geode = new osg::Geode();
    mainNode->addChild(geode.get());

    return mainNode;
}

void GridMapVisualization::updateMainNode ( osg::Node* node )
{
    if(!p) return;

    // Draw map.
    p->visualize(*geode, showHeightField, interpolateCellColors, useNPOTTextures);

    // Draw map extents.
    visualizeMapExtents(p->getCellExtents(), p->getResolution());

    // Set local frame.
    setLocalFrame(p->getLocalFrame());
}

void GridMapVisualization::setShowHeightField(bool enabled)
{
    showHeightField = enabled;
    emit propertyChanged("showHeightField");
    setDirty();
}

bool GridMapVisualization::isHeightFieldShown() const
{
    return showHeightField;
}

void GridMapVisualization::setInterpolateCellColors(bool enabled)
{
    interpolateCellColors = enabled;
    emit propertyChanged("interpolateCellColors");
    setDirty();
}

bool GridMapVisualization::areCellColorsInterpolated() const
{
    return interpolateCellColors;
}

void GridMapVisualization::setUseNPOTTextures(bool enabled)
{
    useNPOTTextures = enabled;
    emit propertyChanged("useNPOTTextures");
    setDirty();
}

bool GridMapVisualization::areNPOTTexturesUsed() const
{
    return useNPOTTextures;
}

void GridMapVisualization::updateDataIntern(::maps::grid::GridMap<double> const& value)
{
    p.reset(new DataHold<double>( value ));
}

void GridMapVisualization::updateDataIntern(::maps::grid::GridMap<float> const& value)
{
    p.reset(new DataHold<float>( value ));
}

void GridMapVisualization::updateDataIntern(::maps::grid::GridMap<int> const& value)
{
    p.reset(new DataHold<int>( value ));
}

void GridMapVisualization::updateDataIntern(::maps::grid::GridMap<char> const& value)
{
    p.reset(new DataHold<char>( value ));
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(ElevationMapVisualization)
