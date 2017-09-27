#include <iostream>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/Tools.hpp>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BOOST_TEST_MODULE "Attenuation_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;

BOOST_AUTO_TEST_SUITE(test_NormalDepthMap)

BOOST_AUTO_TEST_CASE(attenuationCalculation_testCase){
    double frequency = 700.0;   // kHz
    double temperature = 20.0;  // celsius degrees
    double depth = 1;           // meters
    double salinity = 35;       // ppt
    double acidity = 8.1;       // pH

    double attenuationCoeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);
    BOOST_CHECK_CLOSE(attenuationCoeff, 0.0247, 3);
}

// simple sonar plot
void plotSonarTest(cv::Mat3f image, double maxRange, double maxAngleX) {
    cv::Mat1b imagePlot = cv::Mat1b::zeros(500, 500);
    cv::Point2f centerPlot(imagePlot.cols / 2, 0);
    double factor = imagePlot.rows / maxRange;
    double slope = 2 * maxAngleX * (1.0 / (image.cols - 1));

    for (int j = 0; j < image.rows; ++j) {
        for (int i = 0; i < image.cols; ++i) {
            double distance = image[j][i][1] * maxRange;
            double alpha = slope * i - maxAngleX;

            cv::Point2f tempPoint(distance * sin(alpha), distance * cos(alpha));
            tempPoint = tempPoint * factor + centerPlot;
            imagePlot[(uint) tempPoint.y][(uint) tempPoint.x] = 255 * image[j][i][0];
        }
    }

    cv::Mat3b imagePlotMap;
    cv::applyColorMap(imagePlot, imagePlotMap, cv::COLORMAP_HOT);

    cv::line( imagePlotMap, centerPlot, cv::Point2f(maxRange * sin(maxAngleX) * factor,
              maxRange * cos(maxAngleX) * factor) + centerPlot, cv::Scalar(255), 1, CV_AA);

    cv::line( imagePlotMap, centerPlot, cv::Point2f(maxRange * sin(-maxAngleX) * factor,
              maxRange * cos(maxAngleX) * factor) + centerPlot, cv::Scalar(255), 1, CV_AA);

    cv::imshow("Normal Depth Map", image);
    cv::imshow("Sonar Plot Test", imagePlotMap);
    cv::waitKey();
}

// compute the normal depth map for a osg scene
cv::Mat computeNormalDepthMap(  osg::ref_ptr<osg::Group> root,
                                uint height,
                                float maxRange,
                                float fovX,
                                float fovY,
                                double attenuationCoeff,
                                osg::Vec3d eye,
                                osg::Vec3d center,
                                osg::Vec3d up
                            ) {
    // normal depth map
    NormalDepthMap normalDepthMap(maxRange, fovX * 0.5, fovY * 0.5, attenuationCoeff);
    ImageViewerCaptureTool capture(fovY, fovX, height);
    capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
    capture.setCameraPosition(eye, center, up);
    normalDepthMap.setDrawNormal(true);
    normalDepthMap.setDrawDepth(true);
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

// draw the scene with a small ball in the center with a big cube, cylinder and cone in back
void makeSimpleScene(osg::ref_ptr<osg::Group> root) {
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
}

// define different point of views of the same scene
void viewPointsFromScene(std::vector<osg::Vec3d> *eyes,
                          std::vector<osg::Vec3d> *centers,
                          std::vector<osg::Vec3d> *ups) {

    // point of view 1 - near from the ball with the cylinder in back
    eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
    centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
    ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));

    // point of view 2 - near from the ball with the cube in back
    eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
    centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
    ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));

    // point of view 3 - near the cone in up side
    eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
    centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
    ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));

    // point of view 4 - Faced the cube plane
    eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
    centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
    ups->push_back(osg::Vec3d(0, 0, 1));
}

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {
    // sonar parameters
    float maxRange = 50;            // 50 meters
    float fovX = M_PI * 1.0 / 6;    // 30 degrees
    float fovY = M_PI * 1.0 / 6;    // 30 degrees
    uint height = 500;

    // attenuation coefficient
    double frequency = 700.0;   // kHz
    double temperature = 20.0;  // celsius degrees
    double depth = 1;           // meters
    double salinity = 0;        // ppt
    double acidity = 8;         // pH
    double attenuationCoeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);

     // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromScene(&eyes, &centers, &ups);

    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> root = new osg::Group();
    makeSimpleScene(root);

    // display the same with and without underwater acoustic attenuation
    for (uint i = 0; i < eyes.size(); ++i) {
        cv::Mat raw_shader = computeNormalDepthMap(root, height, maxRange, fovX, fovY, 0, eyes[i], centers[i], ups[i]);
        cv::Mat att_shader = computeNormalDepthMap(root, height, maxRange, fovX, fovY, attenuationCoeff, eyes[i], centers[i], ups[i]);
        plotSonarTest(raw_shader, maxRange, fovX * 0.5);
        plotSonarTest(att_shader, maxRange, fovX * 0.5);
  }
}

BOOST_AUTO_TEST_SUITE_END();
