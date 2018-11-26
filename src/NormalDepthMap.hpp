#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_

#include "Tools.hpp"

// C++ includes
#include <vector>

// OSG includes
#include <osg/Node>
#include <osg/Group>
#include <osg/ref_ptr>

namespace normal_depth_map {

/**
 * Gets the informations of normal and distance from a osg scene, between the objects and the camera.
 */
class NormalDepthMap {
public:
    /**
     * Build a map from the normal surface and the distance from objects to the viewer camera by applying shaders.
     * BLUE CHANNEL, presents the normal values from the objects to the center camera, where:
     *      1 is the max value, and represents the normal vector of the object surface and the normal vector of camera are in the same directions;
     *      0 is the minimum value, the normal vector of the object surface and the normal vector of camera are in the perpendicular directions.
     * GREEN CHANNEL presents the distance values relative from camera center, where:
     *      0 is the minimum value, and represents the object is near from the camera;
     *      1 is the max value, and represents the object is far from the camera, and it is limited by max range.
    */
    NormalDepthMap();
    NormalDepthMap(float maxRange);
    NormalDepthMap(float maxRange, float attenuationCoeff);

    /**
     * Add the models in the main normal depth map node
     * @param node: osg node to add a main scene
     */
    void addNodeChild(osg::ref_ptr<osg::Node> node);

    osg::ref_ptr<osg::Group> getNormalDepthMapNode() const {
        return _normalDepthMapNode;
    };

    void setMaxRange(float value) {
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("farPlane")
            ->set(value);
    }

    float getMaxRange() const {
        float value = 0;
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("farPlane")
            ->get(value);
        return value;
    }

    void setAttenuationCoefficient(float value) {
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("attenuationCoeff")
            ->set(value);
    }

    float getAttenuationCoefficient() const {
        float value = 0;
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("attenuationCoeff")
            ->get(value);
        return value;
    }

    void setDrawNormal(bool value) {
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawNormal")
            ->set(value);
    }

    bool isDrawNormal() const {
        bool value;
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawNormal")
            ->get(value);
        return value;
    }

    void setDrawDepth(bool value) {
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawDepth")
            ->set(value);
    }

    bool isDrawDepth() const {
        bool value;
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawDepth")
            ->get(value);
        return value;
    }

    void setDrawReverb(bool value) {
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawReverb")
            ->set(value);
    }

    bool isDrawReverb() const {
        bool value;
        _normalDepthMapNode->getOrCreateStateSet()
            ->getUniform("drawReverb")
            ->get(value);
        return value;
    }

  private:
    //main scene node
    osg::ref_ptr<osg::Group> _normalDepthMapNode;

    /**
     * Setup the main scene node.
     *
     * @param maxRange: limits the maximum observable distance by viewer camera.
     * @param attenuationCoeff: the underwater signal attenuation value.
     * @param drawDepth: enables the distance calculation on shader.
     * @param drawNormal: enables the normal calculation on shader.
     * @param drawReverb: enables the reverberation effect on shader.
     */
    osg::ref_ptr<osg::Group> createTheNormalDepthMapShaderNode(
                              float maxRange = 50.0,
                              float attenuationCoeff = 0,
                              bool drawDepth = true,
                              bool drawNormal = true,
                              bool drawReverb = true);
};
}

#endif
