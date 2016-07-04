#ifndef __VIZKIT_PATCHESGEODE_HPP__
#define __VIZKIT_PATCHESGEODE_HPP__

#include <osg/Geometry>
#include <osg/Geode>

#include <iostream>

namespace vizkit3d {

    class PatchesGeode : public osg::Geode
    {
    public:
        PatchesGeode(float x_res, float y_res);

        void setPosition(float x, float y)
        {
            xp = x; yp = y;
        }

        /** Draws a plane inside the box given by \c position and \c extends,
         *  using \c normal and \c mean (relative to the origin of the box)
         */
        void drawPlane(
            const float& z,
            const float& height,
            const osg::Vec3& mean,
            const osg::Vec3& normal);

        void drawPlane(
            const osg::Vec3& position, 
            const osg::Vec4& heights,
            const osg::Vec3& extents, 
            const osg::Vec3& normal,
            double min,
            double max);

        void drawBox(
            const float& top,
            const float& height,
            const osg::Vec3& c_normal );

        void setColor(const osg::Vec4& color);
        void setColorHSVA(float hue, float sat, float lum, float alpha);

        void showNegative(bool show_negative);
        void setNegativeColor(const osg::Vec4& color);

        void showCycleColor(bool cycle_color);
        void setCycleColorInterval(float cycle_color_interval);

        void setShowPatchExtents(bool enable = true) { showPatchExtents = enable; };
        void setShowNormals(bool enable = true) { showNormals = enable; };
        void drawLines();



    private:
        osg::ref_ptr<osg::Vec3Array> vertices;
        osg::ref_ptr<osg::Vec3Array> normals;
        osg::ref_ptr<osg::Vec4Array> colors;  
        osg::ref_ptr<osg::Geometry> geom;  

        osg::ref_ptr<osg::Vec3Array> var_vertices;

        size_t vertex_index;

        float xp, yp; // position of current patch
        float xs, ys; // grid resolution

        float hue;
        float sat; 
        float alpha; 
        float lum;
        osg::Vec4 color; 

        bool showNormals;
        bool showPatchExtents;
        bool cycle_color;
        float cycle_color_interval;       

        void addVertex(const osg::Vec3& p, const osg::Vec3& n);
        void updateColor();
        
        void closePolygon();
        void closeQuads();
    };

}

#endif // __VIZKIT_PATCHESGEODE_HPP__
