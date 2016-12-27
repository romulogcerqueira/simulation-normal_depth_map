#define BOOST_TEST_MODULE "Material_test"
#include <boost/test/unit_test.hpp>

// OpenSceneGraph includes
#include <osg/AlphaFunc>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Image>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

// Rock includes
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

// C++ includes
#include <iostream>

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(MaterialProperties)

// check if two matrixes are equals
bool are_equals (const cv::Mat& image1, const cv::Mat& image2) {
    cv::Mat diff = image1 != image2;
    return (cv::countNonZero(diff) == 0);
}

// create simple scene without texture
osg::ref_ptr<osg::Group> createSimpleScene(float reflectance) {
    osg::ref_ptr<osg::Group> root = new osg::Group();

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,-14), 5)));
    root->addChild(geode);
    root->getChild(0)->asGeode()->addDrawable(geode->getDrawable(0));

    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("reflectance", reflectance));
    root->setStateSet(stateset);

    return root;
}

// compute the normal depth map for a osg scene
cv::Mat computeNormalDepthMap(osg::ref_ptr<osg::Group> root, float maxRange, float fovX, float fovY) {
    uint height = 500;

    // normal depth map
    NormalDepthMap normalDepthMap(maxRange, fovX * 0.5, fovY * 0.5);
    ImageViewerCaptureTool capture(fovY, fovX, height);
    capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
    normalDepthMap.addNodeChild(root);

    // grab scene
    osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
    osg::ref_ptr<osg::Image> osgDepth = capture.getDepthBuffer();
    cv::Mat cvImage = cv::Mat(osgImage->t(), osgImage->s(), CV_32FC3, osgImage->data());
    cv::Mat cvDepth = cv::Mat(osgDepth->t(), osgDepth->s(), CV_32FC1, osgDepth->data());
    cvDepth = cvDepth.mul( cv::Mat1f(cvDepth < 1) / 255);

    std::vector<cv::Mat> channels;
    cv::split(cvImage, channels);
    channels[1] = cvDepth;
    cv::merge(channels, cvImage);
    cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR);
    cv::flip(cvImage, cvImage, 0);

    return cvImage.clone();
}

BOOST_AUTO_TEST_CASE(differentMaterials_testCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> root1 = createSimpleScene(1.0);
    cv::Mat scene1 = computeNormalDepthMap(root1, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root2 = createSimpleScene(0.35);
    cv::Mat scene2 = computeNormalDepthMap(root2, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root3 = createSimpleScene(1.40);
    cv::Mat scene3 = computeNormalDepthMap(root3, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root4 = createSimpleScene(2.12);
    cv::Mat scene4 = computeNormalDepthMap(root4, maxRange, fovX, fovY);

    std::vector<cv::Mat> channels1, channels2, channels3, channels4;
    cv::split(scene1, channels1);
    cv::split(scene2, channels2);
    cv::split(scene3, channels3);
    cv::split(scene4, channels4);

    // assert that normal matrixes are differents
    BOOST_CHECK(are_equals(channels1[0], channels2[0]) == false);
    BOOST_CHECK(are_equals(channels1[0], channels3[0]) == false);
    BOOST_CHECK(are_equals(channels1[0], channels4[0]) == false);
    BOOST_CHECK(are_equals(channels2[0], channels3[0]) == false);
    BOOST_CHECK(are_equals(channels2[0], channels4[0]) == false);
    BOOST_CHECK(are_equals(channels3[0], channels4[0]) == false);

    // output
    cv::Mat output1, output2, output;
    cv::hconcat(scene1, scene2, output1);
    cv::hconcat(scene3, scene4, output2);
    cv::vconcat(output1, output2, output);
    cv::resize(output, output, cv::Size(output.rows / 2, output.cols / 2));

    cv::imshow("material test", output);
    cv::waitKey();
}

BOOST_AUTO_TEST_SUITE_END()
