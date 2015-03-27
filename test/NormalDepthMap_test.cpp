/*
 * NormalDepthMap_test.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: tiagotrocoli
 */

#include "../src/NormalDepthMap.hpp"

#include <iostream>
#include <osg/Geode>
#include <osg/Group>
#include <osg/ref_ptr>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <osg/Vec3f>
#include <osgViewer/Viewer>

#define BOOST_TEST_MODULE "NormalDepthMap_test"
//VERY IMPORTANT - include this last

#include <boost/test/unit_test.hpp>

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(vizkit3d_NormalDepthMap)

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase1) {

    NormalDepthMap normalDepthMap(50);

    osg::ref_ptr<osg::Group> root = new osg::Group();

    osg::Geode *sphere = new osg::Geode();
    sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
    root->addChild(sphere);

    osg::Geode *cylinder = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
    root->addChild(cylinder);

    osg::Geode *cone = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
    root->addChild(cone);

    osg::Geode *box = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
    root->addChild(box);

    osgViewer::Viewer viewer;
    viewer.setSceneData(normalDepthMap.applyShaderNormalDepthMap(root));
    viewer.run();
}

BOOST_AUTO_TEST_SUITE_END();
