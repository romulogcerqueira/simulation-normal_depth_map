// C++ includes
#include <iostream>

// Rock includes
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include "TestHelper.hpp"

// OSG includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#define BOOST_TEST_MODULE "NormalDepthMap_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(test_NormalDepthMap)

// reference points, and map values for each view in viewPointsFromScene1
void referencePointsFromScene(
        std::vector<std::vector<cv::Point> > *setPoints,
        std::vector<std::vector<cv::Point2f> > *setValues) {

    std::vector<cv::Point> points;
    // image points in view1
    points.push_back(cv::Point(74, 417));
    points.push_back(cv::Point(60, 320));
    points.push_back(cv::Point(267, 130));
    points.push_back(cv::Point(366, 226));
    points.push_back(cv::Point(361, 240));
    points.push_back(cv::Point(424, 314));
    setPoints->push_back(points);
    points.clear();

    // image points in view2
    points.push_back(cv::Point(80, 80));
    points.push_back(cv::Point(130, 475));
    points.push_back(cv::Point(390, 128));
    points.push_back(cv::Point(391, 210));
    points.push_back(cv::Point(280, 187));
    setPoints->push_back(points);
    points.clear();

    // image points in view3
    points.push_back(cv::Point(142, 77));
    points.push_back(cv::Point(254, 309));
    points.push_back(cv::Point(434, 65));
    points.push_back(cv::Point(123, 26));
    points.push_back(cv::Point(200, 100));
    setPoints->push_back(points);
    points.clear();

    // image points in view4
    points.push_back(cv::Point(75, 64));
    points.push_back(cv::Point(250, 251));
    points.push_back(cv::Point(410, 459));
    points.push_back(cv::Point(15, 485));
    points.push_back(cv::Point(461, 36));
    setPoints->push_back(points);

    std::vector<cv::Point2f> values;

    // pixel value from each point in image from view1
    values.push_back(cv::Point2f(0.9921,0.1853));
    values.push_back(cv::Point2f(0.2705,0.1987));
    values.push_back(cv::Point2f(0.9058,0.6374));
    values.push_back(cv::Point2f(0.9529,0.6047));
    values.push_back(cv::Point2f(0.2666,0.6164));
    values.push_back(cv::Point2f(0.1686,0.9812));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view2
    values.push_back(cv::Point2f(0.0000,0.0000));
    values.push_back(cv::Point2f(0.9098,0.7698));
    values.push_back(cv::Point2f(1.0000,0.1942));
    values.push_back(cv::Point2f(0.4313,0.2047));
    values.push_back(cv::Point2f(0.1490,0.8185));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view3
    values.push_back(cv::Point2f(0.9041,0.4190));
    values.push_back(cv::Point2f(0.7557,0.4866));
    values.push_back(cv::Point2f(0.0000,0.0000));
    values.push_back(cv::Point2f(0.9188,0.4112));
    values.push_back(cv::Point2f(0.8138,0.4316));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view4
    values.push_back(cv::Point2f(0.9646,0.4474));
    values.push_back(cv::Point2f(1.0000,0.4316));
    values.push_back(cv::Point2f(0.9607,0.4486));
    values.push_back(cv::Point2f(0.0000,0.0000));
    values.push_back(cv::Point2f(0.9529,0.4535));
    setValues->push_back(values);
    values.clear();
}

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {
    std::vector<std::vector<cv::Point> > setPoints;
    std::vector<std::vector<cv::Point2f> > setValues;
    referencePointsFromScene(&setPoints, &setValues);

    // sonar parameters
    float maxRange = 50;
    float fovX = M_PI * 1.0 / 6; // 30 degrees
    float fovY = M_PI * 1.0 / 6; // 30 degrees

    // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromDemoScene(&eyes, &centers, &ups);

    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> root = new osg::Group();
    makeDemoScene(root);

    // uint precision = 1000;
    for (uint i = 0; i < eyes.size(); i++) {
        // compute and display the final shader and sonar images
        cv::Mat rawShader = computeNormalDepthMap(root, maxRange,
                                                  fovX, fovY, 0,
                                                  eyes[i], centers[i], ups[i]);
        cv::Mat rawSonar  = drawSonarImage(rawShader, maxRange, fovX * 0.5);
        cv::imshow("shader image", rawShader);
        cv::imshow("sonar image", rawSonar);
        cv::waitKey();

        // start check process
        cv::Mat normalMap, depthMap;
        cv::extractChannel(rawShader, normalMap, 0);
        cv::extractChannel(rawShader, depthMap,  1);

        for (uint j = 0; j < setPoints[i].size(); ++j) {
            cv::Point p = setPoints[i][j];

            // check normal values
            BOOST_CHECK_CLOSE(normalMap.at<float>(p.y,p.x),
                              setValues[i][j].x, 2);

            // check depth values
            BOOST_CHECK_CLOSE(depthMap.at<float>(p.y,p.x),
                              setValues[i][j].y, 2);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END();
