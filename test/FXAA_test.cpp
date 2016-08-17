#include <iostream>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/FXAA.hpp>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BOOST_TEST_MODULE "FXAA_test"
#include <boost/test/unit_test.hpp>

using namespace vizkit3d_normal_depth_map;

void makeSimpleScene(osg::ref_ptr<osg::Group> root) {

  osg::Geode *sphere = new osg::Geode();
  sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
  root->addChild(sphere);

  osg::Geode *cylinder = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
  root->addChild(cylinder);

  osg::Geode *cone = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
  root->addChild(cone);

  osg::Geode *box = new osg::Geode();
  cylinder->addDrawable(
    new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
  root->addChild(box);
}

void viewPointsFromScene(std::vector<osg::Vec3d> *eyes,
                          std::vector<osg::Vec3d> *centers,
                          std::vector<osg::Vec3d> *ups) {

  // view1 - near from the ball with the cylinder in back
  eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
  centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
  ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));

  // view2 - near from the ball with the cube in back
  eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
  centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
  ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));

  // view3 - near the cone in up side
  eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
  centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
  ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));

  // view4 - Faced the cube plane
  eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
  centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
  ups->push_back(osg::Vec3d(0, 0, 1));
}

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {

  std::vector<osg::Vec3d> eyes, centers, ups;

  float maxRange = 50;
  float maxAngleX = M_PI * 1.0 / 6; // 30 degrees
  float maxAngleY = M_PI * 1.0 / 6; // 30 degrees

  uint height = 500;
  NormalDepthMap normalDepthMap(maxRange, maxAngleX * 0.5, maxAngleY * 0.5);
  ImageViewerCaptureTool capture(maxAngleY, maxAngleX, height);
  FXAA fxaa;
  capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

  osg::ref_ptr<osg::Group> root = new osg::Group();
  viewPointsFromScene(&eyes, &centers, &ups);
  makeSimpleScene(root);
  normalDepthMap.addNodeChild(root);

  for (uint i = 0; i < eyes.size(); ++i) {
    capture.setCameraPosition(eyes[i], centers[i], ups[i]);

    normalDepthMap.setDrawNormal(true);
    normalDepthMap.setDrawDepth(true);
    osg::ref_ptr<osg::Image> osgImage =
      capture.grabImage(normalDepthMap.getNormalDepthMapNode());
    fxaa.addImageToFxaa(osgImage);

    cv::Mat3f cvImage(osgImage->t(), osgImage->s());
    cvImage.data = osgImage->data();
    cvImage = cvImage.clone();
    cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
    cv::flip(cvImage, cvImage, 0);
    cv::imshow("FXAA OUT",cvImage);
    cv::waitKey();
  }

}
