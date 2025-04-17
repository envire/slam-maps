

// #include <GLES3/gl3.h>
// #include <EGL/egl.h>
// #include <GLFW/glfw3.h>

#include <osg/BlendFunc>

#include <iostream>
#include <osgUtil/SmoothingVisitor>

#include <osg/Uniform>
#include <osgViz/OsgViz.hpp>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/tools/GlobalPos.h>

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 osg_Vertex;\n"
    "layout (location = 1) in vec3 osg_Normal;\n"
    "out vec3 FragPos;\n"
    "out vec3 Normal;\n"
    "uniform mat4 osg_ModelViewProjectionMatrix;\n"
    "uniform mat4 osg_ModelViewMatrix;\n"
    "uniform mat4 osg_ViewMatrixInverse;\n"
    "uniform mat3 osg_NormalMatrix;\n"
    "uniform mat4 osg_ViewMatrix;\n"
    "uniform mat4 modelMatrix;\n"
    "uniform mat4 modelViewProjectionMatrix;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = osg_ModelViewProjectionMatrix * vec4(osg_Vertex, 1.0);\n"
    "    FragPos = vec3(modelMatrix * vec4(osg_Vertex, 1.0));\n"
    "    Normal = osg_NormalMatrix * osg_Normal;\n"
    "}\0";

const char *fragmentShaderSource = "#version 330 core\n"
    "//out vec4 FragColor;\n"
    "in vec3 FragPos;\n"
    "in vec3 Normal;\n"
    "uniform float cycleColorInterval;\n"
    "// from: http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl\n"
    "vec3 hsv2rgb(vec3 c) {\n"
        "c = vec3(c.x, clamp(c.yz, 0.0, 1.0));\n"
        "vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);\n"
        "vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n"
        "return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n"
    "}\n"
    "void main()\n"
    "{\n"
    "   float z = FragPos.z;\n"
    "   float int_part;\n"
    "   if ( abs( modf(z, int_part) ) < 0.02) {\n"
    "      gl_FragColor = vec4(0,0,0,1);"
    "   } else {"
    "      float hue = (z - floor(z / cycleColorInterval) * cycleColorInterval) / cycleColorInterval;\n"
    "      vec3 hsv = vec3(hue, 1.0, 1);\n"
    "      vec3 rgbcolor = hsv2rgb(hsv);\n"
    "      vec3 lightColor = vec3(1.0f, 1.0f, 1.0f);\n"
    "      float ambientStrength = 0.5;\n"
    "      vec3 ambient = ambientStrength * lightColor;\n"
    "      vec3 norm = normalize(Normal);\n"
    "      vec3 lightPos = vec3(10.0 , 0.0, 10.0);\n"
    "      vec3 lightDir = normalize(lightPos - FragPos);\n"
    "      float diff = max(dot(norm, lightDir), 0.0);\n"
    "      vec3 diffuse = diff*lightColor;\n"
    "      vec3 result = (ambient + diffuse) * (0.5 * rgbcolor);" 
    "      gl_FragColor = vec4(result,1);\n"
    "   }"
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

osg::ref_ptr<osg::Program> program = new osg::Program;
osg::ref_ptr<osg::Shader> fShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
osg::ref_ptr<osg::Shader> vShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);



osg::ref_ptr<osg::Uniform> setupShader(osg::ref_ptr<osg::Node> node, osg::Camera* cam, const float& cycleheight) {

 


    node->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

    osg::ref_ptr<osg::Uniform> mvp = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "modelViewProjectionMatrix");
    node->getOrCreateStateSet()->addUniform(mvp);
    mvp->setUpdateCallback(new ModelViewProjectionMatrixCallback(cam));

    osg::ref_ptr<osg::Uniform> model = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "modelMatrix");
    node->getOrCreateStateSet()->addUniform(model);
    model->setUpdateCallback(new ModelMatrixCallback);

    osg::ref_ptr<osg::Uniform> cycleColorIntervalUniform = new osg::Uniform(osg::Uniform::FLOAT, "cycleColorInterval");
    node->getOrCreateStateSet()->addUniform(cycleColorIntervalUniform);
    cycleColorIntervalUniform->set(cycleheight);

    // node->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc(), osg::StateAttribute::ON);


    // node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    // node->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    // node->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);

    return cycleColorIntervalUniform;

}


int main()
{

    program->addShader(vShader);
    program->addShader(fShader);

    osg::ref_ptr<osgviz::OsgViz> osgViz = osgviz::OsgViz::getInstance();
    osgViz->createWindow();
    osg::Camera* cam = osgViz->getWindowManager()->getWindowByID(0)->getView()->getCamera();

    cam->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
    cam->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);

    std::shared_ptr<osgviz::PrimitivesFactory> primitivesfactory = osgviz::OsgViz::getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    osg::ref_ptr<osgviz::Object> grid = primitivesfactory->createGrid();
    osgViz->addChild(grid);


    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::Geometry* geom = new osg::Geometry();

    // https://github.com/openscenegraph/OpenSceneGraph/blob/master/examples/osggeometry/osggeometry.cpp
    // note, anticlockwise ordering.
    osg::Vec3 myCoords[] =
    {
        osg::Vec3(3,0,0),
        osg::Vec3(4,0,0),
        osg::Vec3(4,0,1),
        osg::Vec3(3,0,1),

        osg::Vec3(3,0,0),
        osg::Vec3(3,1,0),
        osg::Vec3(3,1,1),
        osg::Vec3(3,0,1)

    };
    int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);

    osg::Vec3Array* vertices = new osg::Vec3Array(numCoords,myCoords);

    geom->setVertexArray(vertices);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,numCoords));
    geode->addDrawable(geom);
    osgViz->addChild(geode);


    osgUtil::SmoothingVisitor::smooth(*geom);   
    osg::ref_ptr<osg::Uniform> cycleColorIntervalUniform = setupShader(geode, cam, 1);

 


    // osg::ref_ptr<osgviz::PrimitivesFactory::Shape> box = primitivesfactory->createShape(osgviz::PrimitivesFactory::BOX, 0.25,0.25,2);
    // box->setPosition(1,1,1);
    // osgViz->addChild(box);
    // setupShader(box, cam, 1);

    // osg::ref_ptr<osgviz::PrimitivesFactory::Shape> box2 = primitivesfactory->createShape(osgviz::PrimitivesFactory::BOX, 0.25,0.25,2);
    // box2->setPosition(1.25,1.25,1);
    // osgViz->addChild(box2);
    // setupShader(box2, cam, 1);


    while (!osgViz->done()){



        osgViz->update();
    }

}