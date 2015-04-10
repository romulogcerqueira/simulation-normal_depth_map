/*
 * NormalDepthMap.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: tiagotrocoli
 */

#include "NormalDepthMap.hpp"

#include <osg/Node>
#include <osg/Program>
#include <osg/ref_ptr>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osgDB/FileUtils>

namespace vizkit3d_normal_depth_map {

#define SHADER_PATH_FRAG "vizkit3d_normal_depth_map/shaders/normalDepthMap.frag"
#define SHADER_PATH_VERT "vizkit3d_normal_depth_map/shaders/normalDepthMap.vert"

NormalDepthMap::NormalDepthMap(float maxRange) :
        _drawDepth(true), _drawNormal(true) {
    this->setMaxRange(maxRange);
    createTheNormalDepthMapShaderNode();
}

NormalDepthMap::NormalDepthMap() :
        _maxRange(50.0), _drawDepth(true), _drawNormal(true) {
    createTheNormalDepthMapShaderNode();
}

void NormalDepthMap::setMaxRange(double maxRange) {
    _maxRange = abs(maxRange);
    if (_normalDepthMapNode != NULL)
        _normalDepthMapNode->getOrCreateStateSet()->getUniform("farPlane")->set(_maxRange);
}

double NormalDepthMap::getMaxRange() {
    return _maxRange;
}

void NormalDepthMap::setDrawNormal(bool drawNormal) {
    _drawNormal = drawNormal;
    if (_normalDepthMapNode != NULL)
        _normalDepthMapNode->getOrCreateStateSet()->getUniform("drawNormal")->set(_drawNormal);
}

bool NormalDepthMap::isDrawNormal() {
    return _drawNormal;
}

void NormalDepthMap::setDrawDepth(bool drawDepth) {
    _drawDepth = drawDepth;
    if (_normalDepthMapNode != NULL)
        _normalDepthMapNode->getOrCreateStateSet()->getUniform("drawDepth")->set(_drawDepth);
}

void NormalDepthMap::addNodeChild(osg::ref_ptr<osg::Node> node) {
    _normalDepthMapNode->addChild(node);
}

bool NormalDepthMap::isDrawDepth() {
    return _drawDepth;
}

void NormalDepthMap::createTheNormalDepthMapShaderNode() {

    _normalDepthMapNode = new osg::Group();
    osg::ref_ptr<osg::Program> program(new osg::Program());

    //reads the shaders files
    osg::ref_ptr<osg::Shader> shaderVertex = osg::Shader::readShaderFile(osg::Shader::VERTEX, osgDB::findDataFile(SHADER_PATH_VERT));
    osg::ref_ptr<osg::Shader> shaderFragment = osg::Shader::readShaderFile(osg::Shader::FRAGMENT, osgDB::findDataFile(SHADER_PATH_FRAG));
    program->addShader(shaderFragment);
    program->addShader(shaderVertex);

    osg::ref_ptr<osg::StateSet> ss = _normalDepthMapNode->getOrCreateStateSet();
    ss->setAttribute(program);

    //input variables to change shader process
    osg::ref_ptr<osg::Uniform> farPlaneUniform(new osg::Uniform("farPlane", _maxRange));
    ss->addUniform(farPlaneUniform);
    osg::ref_ptr<osg::Uniform> drawNormalUniform(new osg::Uniform("drawNormal", _drawNormal));
    ss->addUniform(drawNormalUniform);
    osg::ref_ptr<osg::Uniform> drawDepthUniform(new osg::Uniform("drawDepth", _drawDepth));
    ss->addUniform(drawDepthUniform);
}

}
