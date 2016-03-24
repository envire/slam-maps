#include <iostream>

#include "ElevationMapVisualization.hpp"

#include <vizkit3d/ColorConversionHelper.hpp>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>

using namespace vizkit3d;
using namespace ::maps;

struct ElevationMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    ::maps::ElevationMap data;
};


ElevationMapVisualization::ElevationMapVisualization()
    : p(new Data)
{
    grid_keys.append(QString("elevation"));
    grid_keys.append(QString("elevation_min"));

    current_grid_key = QString("elevation");

    this->heatMapGradient.createDefaultHeatMapGradient();
}

ElevationMapVisualization::~ElevationMapVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> ElevationMapVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void ElevationMapVisualization::updateMainNode ( osg::Node* node )
{
    // create height field
    osg::ref_ptr<osg::HeightField> heightField = createHeighField();

    // remove old drawables
    osg::Geode* geode = static_cast<osg::Geode*>(node);    
    while(geode->removeDrawables(0));
    // add height field to geode
    osg::ShapeDrawable *drawable = new osg::ShapeDrawable(heightField);
    geode->addDrawable(drawable);    

    // set material properties
    osg::StateSet* state = geode->getOrCreateStateSet();
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

    //state->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
}

osg::HeightField* ElevationMapVisualization::createHeighField()
{
    // create height field
    ElevationMap& elev_map = p->data;    

    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(elev_map.getNumCells().x(), elev_map.getNumCells().y());
    heightField->setXInterval(elev_map.getResolution().x());
    heightField->setYInterval(elev_map.getResolution().y());
    double offset_x = elev_map.translation().x();
    double offset_y = elev_map.translation().y();
    double offset_z = elev_map.translation().z();
    heightField->setOrigin(osg::Vec3d(offset_x, offset_y, offset_z));
    heightField->setSkirtHeight(0.0f); 

    std::pair<double, double> elev_range;
    double default_value;
    if (current_grid_key == QString("elevation_min"))
    {
        elev_range = elev_map.getElevationMinRange();
        default_value = ElevationData::ELEVATION_MIN_DEFAULT;
    }
    else {
        elev_range = elev_map.getElevationRange();
        default_value = ElevationData::ELEVATION_DEFAULT;
    }

    for (unsigned int r = 0; r < heightField->getNumRows(); r++) 
    {
        for (unsigned int c = 0; c < heightField->getNumColumns(); c++) 
        {
            double cell_value;
            if (current_grid_key == QString("elevation_min"))
                cell_value = elev_map.at(Index(c, r)).elevation_min;
            else
                cell_value = elev_map.at(Index(c, r)).elevation;

            if( cell_value !=  default_value)
                heightField->setHeight(c, r, cell_value);
            else
                heightField->setHeight(c, r, elev_range.first);    // min elevation
        }
    }     

    return heightField;  
}

osg::Image* ElevationMapVisualization::createTextureImage()
{
    ElevationMap& elev_map = p->data; 

    osg::Image* image = new osg::Image(); 

    //convert double to uint16 
    int size = elev_map.getNumCells().x() * elev_map.getNumCells().y() * 4;
    unsigned char* image_raw_data = new unsigned char[size];
    unsigned char* pos = image_raw_data;

    std::pair<double, double> elev_range;
    if (current_grid_key == QString("elevation_min"))
    {
        elev_range = elev_map.getElevationMinRange();
    }
    else 
    {
        elev_range = elev_map.getElevationRange();
    }

    //scaling between SCALING_MIN_VALUE and SCALING_MAX_VALUE meters 
    double scaling = std::abs(elev_range.second - elev_range.first);

    if(scaling == 0)
        scaling = 1.0;

    // fill image with color
    for (unsigned int y = 0; y < elev_map.getNumCells().y(); ++y)
    {
        for (unsigned int x = 0; x < elev_map.getNumCells().x(); ++x)
        {
            /** Get the cell value **/
            double cell_value;
            if (current_grid_key == QString("elevation_min"))
                cell_value = elev_map.at(Index(x, y)).elevation_min;
            else
                cell_value = elev_map.at(Index(x, y)).elevation;


            double normalize_value = (cell_value-elev_range.first)/scaling;
            osg::Vec4f col(1.0,1.0,0.6,1.0);
            this->heatMapGradient.getColorAtValue(normalize_value, col.r(),col.g(),col.b());


            *pos++ = (unsigned char)(col.r() * 255.0);
            *pos++ = (unsigned char)(col.g() * 255.0);
            *pos++ = (unsigned char)(col.b() * 255.0);
            *pos++ = (unsigned char)(col.a() * 255.0);
        }
    }

    image->setImage(
            elev_map.getNumCells().x(),
            elev_map.getNumCells().y(),
            1, // datadepth per channel
            GL_RGBA, 
            GL_RGBA, 
            GL_UNSIGNED_BYTE, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
            (unsigned char*)(image_raw_data), // unsigned char* data
            osg::Image::USE_NEW_DELETE, // USE_NEW_DELETE, //osg::Image::NO_DELETE,// AllocationMode mode (shallow copy)
            1);      

    return image;
}

void ElevationMapVisualization::setElevationLayer(const QStringList &keys)
{
    if (keys.empty())
        return;

    setElevationLayer(keys.front());
}

void ElevationMapVisualization::setElevationLayer(const QString &key)
{
    current_grid_key = key;
    emit propertyChanged("elevation_layer");
    setDirty();
}

QString ElevationMapVisualization::getElevationLayer() const
{
    return current_grid_key;
}

QStringList ElevationMapVisualization::getElevationLayers()
{
    if (!current_grid_key.isEmpty() && !grid_keys.isEmpty())
    {
        grid_keys.removeOne(current_grid_key);
        grid_keys.prepend(current_grid_key);
    }

    return grid_keys;
}

void ElevationMapVisualization::updateDataIntern(::maps::ElevationMap const& value)
{
    p->data = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(ElevationMapVisualization)

