#include "NormalDepthMap.hpp"

// C++ includes
#include <iostream>
#include <algorithm>

// OSG includes
#include <osg/Program>
#include <osg/ref_ptr>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osgDB/FileUtils>
#include <osg/ShapeDrawable>

namespace normal_depth_map {

#define SHADER_VERT "normal_depth_map/shaders/shader.vert"
#define SHADER_FRAG "normal_depth_map/shaders/shader.frag"

NormalDepthMap::NormalDepthMap(float maxRange ) {
    _normalDepthMapNode = createTheNormalDepthMapShaderNode(maxRange);
}

NormalDepthMap::NormalDepthMap(float maxRange, float attenuationCoeff) {
    _normalDepthMapNode = createTheNormalDepthMapShaderNode(
                                                          maxRange,
                                                          attenuationCoeff);
}

NormalDepthMap::NormalDepthMap() {
    _normalDepthMapNode = createTheNormalDepthMapShaderNode();
}

void NormalDepthMap::addNodeChild(osg::ref_ptr<osg::Node> node) {
    _normalDepthMapNode->addChild(node);

    // collect all triangles of scene
    TrianglesVisitor trv;
    _normalDepthMapNode->accept(trv);

    // sort the scene's triangles in ascending order (for each model)
    std::vector<Triangle> triangles = trv.getTriangles();
    std::vector<uint> trianglesRef = trv.getTrianglesRef();
    std::vector<BoundingBox> bboxes = trv.getBoundingBoxes();

    // convert triangles (data + reference) to osg texture
    osg::ref_ptr<osg::Texture2D> trianglesTexture;
    triangles2texture(triangles, trianglesRef, bboxes, trianglesTexture);

    // pass the triangles (data + reference) to GLSL as uniform
    osg::ref_ptr<osg::StateSet> ss = _normalDepthMapNode->getChild(0)->getOrCreateStateSet();

    ss->addUniform(new osg::Uniform(osg::Uniform::SAMPLER_2D, "trianglesTex"));
    ss->setTextureAttributeAndModes(0, trianglesTexture, osg::StateAttribute::ON);

    ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC4, "trianglesTexSize"));
    ss->getUniform("trianglesTexSize")->set(osg::Vec4(  triangles.size() * 1.0,
                                                                (triangles.size() + trianglesRef.size()) * 1.0,
                                                                trianglesTexture->getTextureWidth() * 1.0,
                                                                trianglesTexture->getTextureHeight() * 1.0));
}

osg::ref_ptr<osg::Group> NormalDepthMap::createTheNormalDepthMapShaderNode(
                                                float maxRange,
                                                float attenuationCoeff,
                                                bool drawDepth,
                                                bool drawNormal,
                                                bool drawReverb) {

    // setup connection between OSG and shaders
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::Program> program = new osg::Program();
    osg::ref_ptr<osg::StateSet> ss = root->getOrCreateStateSet();
    program->addShader(osg::Shader::readShaderFile(osg::Shader::VERTEX,     osgDB::findDataFile(SHADER_VERT)));
    program->addShader(osg::Shader::readShaderFile(osg::Shader::FRAGMENT,   osgDB::findDataFile(SHADER_FRAG)));
    ss->setAttributeAndModes( program, osg::StateAttribute::ON );

    // set uniforms
    ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT, "farPlane"));
    ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT, "attenuationCoeff"));
    ss->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawDistance"));
    ss->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawNormal"));
    ss->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawReverb"));
    ss->getUniform("farPlane")->set(maxRange);
    ss->getUniform("attenuationCoeff")->set(attenuationCoeff);
    ss->getUniform("drawDistance")->set(drawDepth);
    ss->getUniform("drawNormal")->set(drawNormal);
    ss->getUniform("drawReverb")->set(drawReverb);

    return root;
}

} // namespace normal_depth_map
