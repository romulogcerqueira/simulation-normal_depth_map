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

#define SHADER_PATH_FRAG "normal_depth_map/shaders/normalDepthMap.frag"
#define SHADER_PATH_VERT "normal_depth_map/shaders/normalDepthMap.vert"

NormalDepthMap::NormalDepthMap(float maxRange) :
        maxRange(50.0), drawDepth(true), drawNormal(true) {
    this->setMaxRange(maxRange);
}

NormalDepthMap::NormalDepthMap() :
        maxRange(50.0), drawDepth(true), drawNormal(true) {
    // TODO Auto-generated constructor stub
}

NormalDepthMap::~NormalDepthMap() {
    // TODO Auto-generated destructor stub
}

osg::ref_ptr<osg::Group> NormalDepthMap::applyShaderNormalDepthMap(osg::ref_ptr<osg::Node> node) {

    osg::ref_ptr<osg::Group> localRoot = new osg::Group();

    osg::ref_ptr<osg::Program> program(new osg::Program());

    osg::ref_ptr<osg::Shader> shaderVertex = osg::Shader::readShaderFile(osg::Shader::VERTEX, osgDB::findDataFile(SHADER_PATH_VERT));
    osg::ref_ptr<osg::Shader> shaderFragment = osg::Shader::readShaderFile(osg::Shader::FRAGMENT, osgDB::findDataFile(SHADER_PATH_FRAG));
    program->addShader(shaderFragment);
    program->addShader(shaderVertex);

    osg::ref_ptr<osg::StateSet> ss = localRoot->getOrCreateStateSet();
    ss->setAttribute(program);

    osg::ref_ptr<osg::Uniform> farPlaneUniform(new osg::Uniform("farPlane", this->maxRange));
    ss->addUniform(farPlaneUniform);
    osg::ref_ptr<osg::Uniform> drawNormalUniform(new osg::Uniform("drawNormal", this->drawNormal));
    ss->addUniform(drawNormalUniform);
    osg::ref_ptr<osg::Uniform> drawDepthUniform(new osg::Uniform("drawDepth", this->drawDepth));
    ss->addUniform(drawDepthUniform);

    localRoot->addChild(node);

    return localRoot;
}

void NormalDepthMap::setMaxRange(double maxRange) {
    this->maxRange = abs(maxRange);
}

double NormalDepthMap::getMaxRange() {
    return this->maxRange;
}

void NormalDepthMap::setDrawNormal(bool drawNormal) {
    this->drawNormal = drawNormal;
}

double NormalDepthMap::isDrawNormal() {
    return this->drawNormal;
}

void NormalDepthMap::setDrawDepth(bool drawDepth) {
    this->drawDepth = drawDepth;
}

double NormalDepthMap::isDrawDepth() {
    return this->drawDepth;
}

}
