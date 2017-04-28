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

#include "ExtentsRectangle.hpp"

#include "ColorGradient.hpp"

using namespace vizkit3d;
using namespace ::maps::grid;

struct GridMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    virtual ~Data() { }
    virtual void visualize(osg::Geode& geode) const = 0;
    virtual void visualizeExtents(osg::Group* group) const = 0;
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

    void visualize(osg::Geode& geode) const
    {
        // create height field
        osg::ref_ptr<osg::HeightField> heightField = createHeighField();

        while(geode.removeDrawables(0));
        // add height field to geode
        osg::ShapeDrawable *drawable = new osg::ShapeDrawable(heightField);
        geode.addDrawable(drawable);    

        // set material properties
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
        tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        state->setTextureAttributeAndModes(0, tex);          
    };
    
    void visualizeExtents(osg::Group* group) const
    {
        // get the color as a function of the group
        float scale = ((long)group%1000)/1000.0;
        osg::Vec4 col(0,0,0,1);
        vizkit3d::hslToRgb( scale, 1.0, 0.6, col.x(), col.y(), col.z() );
      
        CellExtents extents = grid.calculateCellExtents();

        Eigen::Vector2d min = extents.min().cast<double>().cwiseProduct(grid.getResolution());
        Eigen::Vector2d max = extents.max().cast<double>().cwiseProduct(grid.getResolution());

        Eigen::Vector3d min_d = grid.getLocalFrame().inverse() * Eigen::Vector3d(min.x(), min.y(), 0.);
        Eigen::Vector3d max_d = grid.getLocalFrame().inverse() * Eigen::Vector3d(max.x(), max.y(), 0.);

        group->addChild( 
            new ExtentsRectangle( Eigen::Vector2d(min_d.x(), min_d.y()),
                Eigen::Vector2d(max_d.x(), max_d.y())));
    };

    osg::HeightField* createHeighField() const
    {
        // create height field
        osg::HeightField* heightField = new osg::HeightField();
        heightField->allocate(grid.getNumCells().x(), grid.getNumCells().y());
        heightField->setXInterval(grid.getResolution().x());
        heightField->setYInterval(grid.getResolution().y());
        double offset_x = grid.translation().x();
        double offset_y = grid.translation().y();
        double offset_z = grid.translation().z();
        heightField->setOrigin(osg::Vec3d(offset_x, offset_y, offset_z));
        heightField->setSkirtHeight(0.0f); 

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
    : p(0),
      showMapExtents(true)
{}

GridMapVisualization::~GridMapVisualization()
{
}

osg::ref_ptr<osg::Node> GridMapVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    osg::ref_ptr<osg::Group> group = new osg::Group();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    group->addChild(geode.get());

    return group.release();
}

void GridMapVisualization::updateMainNode ( osg::Node* node )
{
    if(!p) return;

    osg::Group* group = static_cast<osg::Group*>(node);   

    // DRAW MAP
    osg::Geode* geode = new osg::Geode();    
    group->setChild( 0, geode );
    p->visualize(*geode);

    // DRAW EXTENTS
    // draw the extents of the mls
    group->removeChild( 1 );
    if( showMapExtents )
    {
        p->visualizeExtents(group);
    }
}

void GridMapVisualization::setShowMapExtents(bool value)
{
    showMapExtents = value;
    emit propertyChanged("show_map_extents");
    setDirty();
}

bool GridMapVisualization::areMapExtentsShown() const
{
    return showMapExtents;
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
