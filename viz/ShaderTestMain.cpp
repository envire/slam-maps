

// #include <GLES3/gl3.h>
// #include <EGL/egl.h>
// #include <GLFW/glfw3.h>

#include <iostream>

#include <osg/Uniform>
#include <osgViz/OsgViz.hpp>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/tools/GlobalPos.h>

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 position;\n"
    "out vec4 fragPos;\n"
    "uniform mat4 modelMatrix;\n"
    "uniform mat4 modelViewProjectionMatrix;\n"
    "void main()\n"
    "{\n"
    "    fragPos = modelMatrix * vec4(position, 1.0);\n"
    "    gl_Position = modelViewProjectionMatrix * vec4(position, 1.0);\n"
    "}\0";
const char *fragmentShaderSource = "#version 330 core\n"
    "layout (location = 0) out vec4 color;\n"
    "in vec4 fragPos;\n"
    "// from: http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl\n"
    "vec3 hsv2rgb(vec3 c) {\n"
        "c = vec3(c.x, clamp(c.yz, 0.0, 1.0));\n"
        "vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);\n"
        "vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n"
        "return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n"
    "}\n"
    "void main()\n"
    "{\n"
    "   float z = fragPos.z;\n"
    "   float hue = (z - floor(z / 1.0) * 1.0) / 1.0;\n"
    "   vec3 hsv = vec3(hue, 1, 1);\n"
    "   color = vec4(hsv2rgb(hsv),1);\n"
    "}\n\0";


// https://gist.github.com/vicrucann/497fd5839bccba45e58b5ca48feca12f

struct ModelViewProjectionMatrixCallback: public osg::Uniform::Callback
{
    ModelViewProjectionMatrixCallback(osg::Camera* camera) :
            _camera(camera) {
    }

    virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
        osg::Matrixd viewMatrix = _camera->getViewMatrix();
        osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
        osg::Matrixd modelViewProjectionMatrix = modelMatrix * viewMatrix * _camera->getProjectionMatrix();
        uniform->set(modelViewProjectionMatrix);
    }

    osg::Camera* _camera;
};

struct ModelMatrixCallback: public osg::Uniform::Callback
{
    ModelMatrixCallback(){
    }

    virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
        osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
        uniform->set(modelMatrix);
    }

};


// struct ViewMatrixCallback: public osg::Uniform::Callback
// {
//     ViewMatrixCallback(osg::Camera* camera) :
//             _camera(camera) {
//     }

//     virtual void operator()(osg::Uniform* uniform, osg::NodeVisitor* nv) {
//         osg::Matrixd viewMatrix = _camera->getViewMatrix();
//         //osg::Matrixd viewMatrix = _camera->getProjectionMatrix();
//         // osg::Matrixd modelMatrix = osg::computeLocalToWorld(nv->getNodePath());
//         // osg::Matrixd modelViewProjectionMatrix = modelMatrix * viewMatrix * _camera->getProjectionMatrix();
//         uniform->set(viewMatrix);
//     }

//     osg::Camera* _camera;
// };




int main()
{

    osg::ref_ptr<osgviz::OsgViz> osgViz = osgviz::OsgViz::getInstance();
    

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    osg::Geometry* geom = new osg::Geometry();


    // https://github.com/openscenegraph/OpenSceneGraph/blob/master/examples/osggeometry/osggeometry.cpp
    // note, anticlockwise ordering.
    osg::Vec3 myCoords[] =
    {
        osg::Vec3(1, 0, 0),
        osg::Vec3(1,1,2),
        osg::Vec3(0, 1, 0),
        osg::Vec3(0,0,0)

    };
    int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);

    osg::Vec3Array* vertices = new osg::Vec3Array(numCoords,myCoords);

    geom->setVertexArray(vertices);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,numCoords));
    
    geode->addDrawable(geom);

    osg::ref_ptr<osg::Program> program = new osg::Program;
    osg::ref_ptr<osg::Shader> fShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
    osg::ref_ptr<osg::Shader> vShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
    program->addShader(vShader);
    program->addShader(fShader);
    geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);


    osgViz->addChild(geode);



    //use osg::Uniform::Callback?




    std::shared_ptr<osgviz::PrimitivesFactory> primitivesfactory = osgviz::OsgViz::getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    osg::ref_ptr<osgviz::Object> grid = primitivesfactory->createGrid();
    osgViz->addChild(grid);

    osgViz->createWindow();




    // osg::ref_ptr<osg::Uniform> view = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "viewMatrix");
    // geode->getOrCreateStateSet()->addUniform(view);
    // view->setUpdateCallback(new ViewMatrixCallback(osgViz->getWindowManager()->getWindowByID(0)->getView()->getCamera()));
    


    while (!osgViz->done()){



        osgViz->update();
    }

}