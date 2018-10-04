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
 * @brief Gets the informations of normal and depth from a osg scene, between the objects and the camera.
 *
 * Using a shaders render process, it is build a map with two values:
 *  The dot product with normal vector of the object surface and the camera
 *    normal vector, called normal value;
 *  The distance between the objects and the camera, called depth value;
 *
 */
class NormalDepthMap {
public:
    /**
     * @brief Build a map informations from the normal surface and depth from objects to the camera.
     *
     * This class takes a osg node scene and apply the shaders
     *  to get the depth and normal surface information between the
     *  camera and the objects in the scene.
     *
     *  This class apply the shaders to get the normal and depth information, to build the map in osg::Node.
     *  BLUE CHANNEL, presents the normal values from the objects to the center camera, where:
     *      1 is the max value, and represents the normal vector of the object surface and the normal vector of camera are in the same directions, || ;
     *      0 is the minimum value, the normal vector of the object surface and the normal vector of camera are in the perpendicular directions, |_ ;
     *  GREEN CHANNEL presents the linear depth values relative from camera center in 8bits precision, where:
     *      0 is the minimum value, and represents the object is near from the camera;
     *      1 is the max value, and represents the object is far from the camera, and it is limited by max range;
     *  DEPTH BUFFER presents the linear depth values relative from camera center in 32bits precision, where:
     *      0 is the minimum value, and represents the object is near from the camera;
     *      1 is the max value, and represents the object is far from the camera, and it is limited by max range;
     *
     *  @param maxRange: It is a float value which limits the depth calculation process. Default maxRange = 50.0
     */
    NormalDepthMap();
    NormalDepthMap(float maxRange);
    NormalDepthMap(float maxRange, float attenuationCoeff);

    /**
     * @brief Add the models in the normal depth map node
     *  @param node: osg node to add a main scene
     */
    void addNodeChild(osg::ref_ptr<osg::Node> node);

    /**
     * @brief Get the node with the normal and depth map
     *  @param node: It is a node with the models in the target scene
     */
    const osg::ref_ptr<osg::Group> getNormalDepthMapNode() const {
        return _normalDepthMapNode;
    };

    void setMaxRange(float maxRange);
    float getMaxRange();

    void setAttenuationCoefficient(float coefficient);
    float getAttenuationCoefficient();

    void setDrawNormal(bool value);
    bool isDrawNormal();

    void setDrawDepth(bool value);
    bool isDrawDepth();

    void setDrawReverb(bool value);
    bool isDrawReverb();

  private:
    osg::ref_ptr<osg::Group> _normalDepthMapNode; //main shader node

    osg::ref_ptr<osg::Group> createTheNormalDepthMapShaderNode(
                              float maxRange = 50.0,
                              float attenuationCoefficient = 0,
                              bool drawDepth = true,
                              bool drawNormal = true,
                              bool drawReverb = true);
};
}

#endif /* SIMULATION_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_ */
