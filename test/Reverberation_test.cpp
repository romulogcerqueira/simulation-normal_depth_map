// OSG includes
#include <osgViewer/Viewer>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/TriangleFunctor>
#include <osgDB/FileUtils>

// C++ includes
#include <iostream>
#include <vector>

// Rock includes
#include <normal_depth_map/Tools.hpp>
#include "TestHelper.hpp"

#define BOOST_TEST_MODULE "Reverberation_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(Reverberation)

BOOST_AUTO_TEST_CASE(reverberation_testCase) {
    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> scene = new osg::Group();
    makeDemoScene2(scene);

    // sonar parameters
    float maxRange = 15;      // 15 meters
    float fovX = M_PI / 4;    // 45 degrees
    float fovY = M_PI / 4;    // 45 degrees

    // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromDemoScene2(&eyes, &centers, &ups);

    // compute and display the final shader and sonar images
    for (unsigned i = 0; i < eyes.size(); ++i) {
        cv::Mat shaderImg = computeNormalDepthMap(scene, maxRange, fovX, fovY, 0, eyes[i], centers[i], ups[i]);
        cv::Mat sonarImg = drawSonarImage(shaderImg, maxRange, fovX * 0.5);
        cv::imshow("shaderImg", shaderImg);
        cv::imshow("sonarImg", sonarImg);
        cv::waitKey();
    }
}

BOOST_AUTO_TEST_SUITE_END();
