#include <iostream>
#include "NormalDepthMapVisualization.hpp"

#include <osg/Geode>
#include <osg/ShapeDrawable>

using namespace vizkit3d;

struct NormalDepthMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::RigidBodyState data;
};

NormalDepthMapVisualization::NormalDepthMapVisualization() :
        p(new Data), _normalDepthMap() {
}

NormalDepthMapVisualization::~NormalDepthMapVisualization() {
    delete p;
}

osg::ref_ptr<osg::Node> NormalDepthMapVisualization::createMainNode() {
    return _normalDepthMap.getNormalDepthMapNode();
}

void NormalDepthMapVisualization::updateMainNode(osg::Node* node) {
    osg::Geode* geode = static_cast<osg::Geode*>(node);
// Update the main node using the data in p->data
}

void NormalDepthMapVisualization::updateDataIntern(base::samples::RigidBodyState const& value) {
    p->data = value;

}
void vizkit3d::NormalDepthMapVisualization::addNodeChild(osg::ref_ptr<osg::Node> node) {
    _normalDepthMap.addNodeChild(node);
}

void vizkit3d::NormalDepthMapVisualization::setMaxRange(float maxRange) {
    _normalDepthMap.setMaxRange(maxRange);
}

float vizkit3d::NormalDepthMapVisualization::getMaxRange() {
    return _normalDepthMap.getMaxRange();
}

void vizkit3d::NormalDepthMapVisualization::setDrawNormal(bool drawNormal) {
    _normalDepthMap.setDrawNormal(drawNormal);
}

bool vizkit3d::NormalDepthMapVisualization::getDrawNormal() {
    return _normalDepthMap.isDrawNormal();
}

void vizkit3d::NormalDepthMapVisualization::setDrawDepth(bool drawDepth) {
    _normalDepthMap.setDrawDepth(drawDepth);
}

bool vizkit3d::NormalDepthMapVisualization::getDrawDepth() {
    return _normalDepthMap.isDrawDepth();
}
//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(NormalDepthMapVisualization)

