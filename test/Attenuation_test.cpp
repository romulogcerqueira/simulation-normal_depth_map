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


 BOOST_AUTO_TEST_CASE(underwaterAttenuationCompute_testcase){

   double temperature = 20.0; // celsius
   double frequency = 700;    // kHz
   double depth_rate = 1;     // meters

   double attenuation_coeff = underwaterSignalAttenuation(temperature, frequency, depth_rate);
   std::cout << "Attenuation coefficient: " << attenuation_coeff << std::endl;

}




// void plotSonar(cv::Mat3f osgImage, cv::Size size, double maxRange, double maxAngleX, cv::Mat1f cv_depth) {
//     cv::Mat1b sonarImage = cv::Mat1b::zeros(size);
//     cv::Point2f centerImage(osgImage.cols / 2, osgImage.rows / 2);
//     cv::Point2f centerPlot(sonarImage.cols / 2, 0);
//     double factor = sonarImage.rows / maxRange;
//     double pointSize = factor / 3;
//     cv::Point2f halfSize(pointSize / 2, pointSize / 2);
//
//     double slope = 2 * maxAngleX * (1.0 / (osgImage.cols - 1));
//     double constant = - maxAngleX;
//
//     for (int j = 0; j < osgImage.rows; ++j) {
//         for (int i = 0; i < osgImage.cols; ++i) {
//             double distance = cv_depth[j][i] * maxRange;
//             double alpha = slope * i + constant;
//
//             cv::Point2f tempPoint(distance * sin(alpha), distance * cos(alpha));
//             tempPoint = tempPoint * factor;
//             tempPoint += centerPlot;
//             sonarImage[(uint) tempPoint.y][(uint) tempPoint.x] = 255 * osgImage[j][i][0];
//         }
//     }
//
//     cv::Mat3b sonarImageColor;
//     cv::applyColorMap(sonarImage, sonarImageColor, cv::COLORMAP_HOT);
//
//     cv::line(sonarImageColor, centerPlot,
//     cv::Point2f(maxRange * sin(maxAngleX) * factor, maxRange * cos(maxAngleX) * factor) + centerPlot, cv::Scalar(255), 1, CV_AA);
//
//     cv::line(sonarImageColor, centerPlot, cv::Point2f(maxRange * sin(-maxAngleX) * factor, maxRange * cos(maxAngleX) * factor) + centerPlot,
//     cv::Scalar(255), 1, CV_AA);
//
//     cv::imshow("sonar image", sonarImageColor);
//     cv::imshow("shader image", osgImage);
//     cv::waitKey();
// }
//
// // draw the scene with a small ball in the center with a big cube, cylinder and cone in back
// void makeSimpleScene(osg::ref_ptr<osg::Group> root) {
//     osg::Geode *sphere = new osg::Geode();
//     sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
//     root->addChild(sphere);
//
//     osg::Geode *cylinder = new osg::Geode();
//     cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
//     root->addChild(cylinder);
//
//     osg::Geode *cone = new osg::Geode();
//     cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
//     root->addChild(cone);
//
//     osg::Geode *box = new osg::Geode();
//     cylinder->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
//     root->addChild(box);
// }
//
// void viewPointsFromScene(std::vector<osg::Vec3d> *eyes,
//                           std::vector<osg::Vec3d> *centers,
//                           std::vector<osg::Vec3d> *ups) {
//
//     // point of view 1 - near from the ball with the cylinder in back
//     eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
//     centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
//     ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));
//
//     // point of view 2 - near from the ball with the cube in back
//     eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
//     centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
//     ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));
//
//     // point of view 3 - near the cone in up side
//     eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
//     centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
//     ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));
//
//     // point of view 4 - Faced the cube plane
//     eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
//     centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
//     ups->push_back(osg::Vec3d(0, 0, 1));
// }
//
// BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {
//
// std::vector<osg::Vec3d> eyes, centers, ups;
//
// float maxRange = 50;
// float maxAngleX = M_PI * 1.0 / 6; // 30 degrees
// float maxAngleY = M_PI * 1.0 / 6; // 30 degrees
//
// uint height = 500;
// NormalDepthMap normalDepthMap(maxRange, maxAngleX * 0.5, maxAngleY * 0.5);
// ImageViewerCaptureTool capture(maxAngleY, maxAngleX, height);
// capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
//
// osg::ref_ptr<osg::Group> root = new osg::Group();
// viewPointsFromScene(&eyes, &centers, &ups);
// makeSimpleScene(root);
// normalDepthMap.addNodeChild(root);
//
// uint precision = 1000;
//
//   for (uint i = 0; i < eyes.size(); ++i) {
//     capture.setCameraPosition(eyes[i], centers[i], ups[i]);
//
//     normalDepthMap.setDrawNormal(true);
//     normalDepthMap.setDrawDepth(true);
//     osg::ref_ptr<osg::Image> osgImage =
//       capture.grabImage(normalDepthMap.getNormalDepthMapNode());
//     cv::Mat3f cvImage(osgImage->t(), osgImage->s());
//     cvImage.data = osgImage->data();
//     cvImage = cvImage.clone();
//     cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
//     cv::flip(cvImage, cvImage, 0);
//
//     //get only normal map
//     normalDepthMap.setDrawDepth(false);
//     osg::ref_ptr<osg::Image> osgImageNormalMap =
//       capture.grabImage(normalDepthMap.getNormalDepthMapNode());
//     cv::Mat3f cvImageNormalMap(osgImage->t(), osgImage->s());
//     cvImageNormalMap.data = osgImageNormalMap->data();
//     cvImageNormalMap = cvImageNormalMap.clone();
//     cv::cvtColor(cvImageNormalMap, cvImageNormalMap, cv::COLOR_RGB2BGR,
//                  CV_32FC3);
//     cv::flip(cvImageNormalMap, cvImageNormalMap, 0);
//
//     //get only half range depth map;
//     normalDepthMap.setDrawDepth(true);
//     normalDepthMap.setDrawNormal(false);
//     osg::ref_ptr<osg::Image> osgImageDepthMap =
//       capture.grabImage(normalDepthMap.getNormalDepthMapNode());
//     cv::Mat3f cvImageDepthMap(osgImage->t(), osgImage->s());
//     cvImageDepthMap.data = osgImageDepthMap->data();
//     cv::cvtColor(cvImageDepthMap, cvImageDepthMap, cv::COLOR_RGB2BGR, CV_32FC3);
//     cv::flip(cvImageDepthMap, cvImageDepthMap, 0);
//
//     // get linear depth with high resolution
//     osg::ref_ptr<osg::Image> osg_depth =  capture.getDepthBuffer();
//     cv::Mat1f cv_depth(osg_depth->t(), osg_depth->s());
//     cv_depth.data = osg_depth->data();
//     cv_depth = cv_depth.clone();
//     cv::flip(cv_depth, cv_depth, 0);
//     cv_depth = cv_depth.mul( cv::Mat1f(cv_depth < 1)/255);
//     // cv::imshow("DEPTH BUFFER", cv_depth);
//
//
//     //start check process
//     plotSonar(cvImage, cv::Size(800,800), maxRange, maxAngleX * 0.5, cv_depth);
//
//   }
// }
//
// BOOST_AUTO_TEST_CASE(depthValueRadialVariation_testCase) {
//
//   osg::ref_ptr<osg::Geode> scene = new osg::Geode();
//   osg::ref_ptr<osg::Shape> box;
//   uint numberSphere = 100;
//   double multi = 2;
//   double boxSize = 5;
//   double distance = 100;
//   double maxRange = 200;
//
//   for (uint i = 0; i < numberSphere; ++i) {
//     box = new osg::Box(osg::Vec3(i * multi, 0, -distance), boxSize);
//     scene->addDrawable(new osg::ShapeDrawable(box));
//     box = new osg::Box(osg::Vec3(i * -multi, 0, -distance), boxSize);
//     scene->addDrawable(new osg::ShapeDrawable(box));
//     box = new osg::Box(osg::Vec3(0, i * -multi, -distance), boxSize);
//     scene->addDrawable(new osg::ShapeDrawable(box));
//     box = new osg::Box(osg::Vec3(0, i * multi, -distance), boxSize);
//     scene->addDrawable(new osg::ShapeDrawable(box));
//   }
//
//   NormalDepthMap normalDepthMap(maxRange, M_PI / 6, M_PI / 6);
//   normalDepthMap.setDrawNormal(false);
//   normalDepthMap.addNodeChild(scene);
//
//   uint sizeVector = 3;
//   double fovys[] = {150, 120, 20};
//   double fovxs[] = {20, 120, 150};
//   uint heightSize[] = {500, 500, 500};
//
//   for (uint j = 0; j < sizeVector; ++j) {
//     ImageViewerCaptureTool capture(fovys[j] * M_PI / 180.0,
//                                    fovxs[j] * M_PI / 180.0, heightSize[j]);
//     capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
//     osg::ref_ptr<osg::Image> osgImage =
//       capture.grabImage(normalDepthMap.getNormalDepthMapNode());
//
//     cv::Mat3f cvImage(osgImage->t(), osgImage->s());
//     cvImage.data = osgImage->data();
//     cvImage = cvImage.clone();
//     cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
//     cv::flip(cvImage, cvImage, 0);
//   }
// }

BOOST_AUTO_TEST_SUITE_END();
