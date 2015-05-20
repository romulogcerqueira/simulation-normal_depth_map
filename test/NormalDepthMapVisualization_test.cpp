/*
 * NormalDepthMapVisualization_test.cpp
 *
 *  Created on: Apr 13, 2015
 *      Author: tiagotrocoli
 */

#include <QtCore>
#include <QApplication>
#include <iostream>
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include "vizkit3d/NormalDepthMapVisualization.hpp"
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

#define BOOST_TEST_MODULE "NormalDepthMapVisualization_test"
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/unit_test_suite_impl.hpp>

BOOST_AUTO_TEST_SUITE(vizkit3d_NormalDepthMapVisualization)

void makeSimpleScene1(osg::ref_ptr<osg::Group> root) {

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

//    root->addChild(osgDB::readNodeFile("/home/tiagotrocoli/senai/rock/gui/vizkit3d_normal_depth_map/test/cessna.osg"));
}

BOOST_AUTO_TEST_CASE(normalDepthMapVisualization_TestCase) {

    int argc = boost::unit_test::framework::master_test_suite().argc;
    char** argv = boost::unit_test::framework::master_test_suite().argv;

    QApplication app(argc, argv);
    vizkit3d::Vizkit3DWidget *widget = new vizkit3d::Vizkit3DWidget();
    widget->show();

    vizkit3d::NormalDepthMapVisualization* plugin = new vizkit3d::NormalDepthMapVisualization();
    osg::ref_ptr<osg::Group> root = new osg::Group();
    makeSimpleScene1(root);
    plugin->addNodeChild(root);
    widget->addPlugin(plugin);

    app.exec();

}
BOOST_AUTO_TEST_SUITE_END();
