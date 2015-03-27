/*
 * NormalDepthMap.h
 *
 *  Created on: Mar 27, 2015
 *      Author: tiagotrocoli
 */

#ifndef GUI_VIZKIT3D_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_
#define GUI_VIZKIT3D_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_

#include <osg/Node>
#include <osg/ref_ptr>

namespace vizkit3d_normal_depth_map {

class NormalDepthMap {
public:
    NormalDepthMap();
    NormalDepthMap(float maxRange);
    virtual ~NormalDepthMap();

    osg::ref_ptr<osg::Group> applyShaderNormalDepthMap(osg::ref_ptr<osg::Node> node);

    void setMaxRange(double maxRange);
    double getMaxRange();

    void setDrawNormal(bool drawNormal);
    double isDrawNormal();

    void setDrawDepth(bool drawDepth);
    double isDrawDepth();

private:

    // Shader parameters
    float maxRange;
    bool drawDepth;
    bool drawNormal;

};
}

#endif /* GUI_VIZKIT3D_NORMAL_DEPTH_MAP_SRC_NORMALDEPTHMAP_HPP_ */
