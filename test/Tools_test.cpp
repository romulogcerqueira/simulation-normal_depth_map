// C++ includes
#include <iostream>

// Rock includes
#include <normal_depth_map/Tools.hpp>
#include "TestHelper.hpp"

// OSG includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define BOOST_TEST_MODULE "Tools_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(Tools)

std::vector<Triangle> input_triangles;
void triangleStructureDataSet(std::vector<Triangle>& input){

    if(input.size() != 0)
      return;

    input.push_back(Triangle(
      osg::Vec3(0.0, 5.0, 0.0),
      osg::Vec3(-5.0, -5.0, 0.0),
      osg::Vec3(5.0, -5.0, 0.0)));
    input.push_back(Triangle(
      osg::Vec3(-9.87688, 31.5643, -2.5),
      osg::Vec3(-10, 30, -2.5),
      osg::Vec3(-9.5, 30, -2)));
    input.push_back(Triangle(
      osg::Vec3(-1.97538, 30.3129, 5.5),
      osg::Vec3(-2, 30, 5.5),
      osg::Vec3(-1.5, 30, 6)));
    input.push_back(Triangle(
      osg::Vec3(-0.523721, 0.266849, -0.809017),
      osg::Vec3(-0.404509, 0.206107, -0.891007),
      osg::Vec3(-0.559017, 0.181635, -0.809017)));
    input.push_back(Triangle(
      osg::Vec3(-0.110616, -0.110616, -0.987688),
      osg::Vec3(0, 0, -1),
      osg::Vec3(-0.0919497, -0.126558, -0.987688)));

    input.push_back(Triangle(
      osg::Vec3(0.15451, -0.975528, 0.156435),
      osg::Vec3(0.305214, -0.939347, 0.156435),
      osg::Vec3(0.293894, -0.904508, 0.309017)));
    input.push_back(Triangle(
      osg::Vec3(0.769421, -0.249998, 0.587785),
      osg::Vec3(0.847398, -0.275334, 0.453991),
      osg::Vec3(0.799057, -0.126557, 0.587785)));
    input.push_back(Triangle(
      osg::Vec3(5.29007, 37.2812, -1.5),
      osg::Vec3(4.08591, 38.0191, -1.5),
      osg::Vec3(3.85892, 37.5736, -1)));
    input.push_back(Triangle(
      osg::Vec3(-200.5, 2.5, -97.5),
      osg::Vec3(-200.5, -2.5, -102.5),
      osg::Vec3(-200.5, -2.5, -97.5)));
    input.push_back(Triangle(
      osg::Vec3(-118.5, -2.5, -97.5),
      osg::Vec3(-113.5, -2.5, -102.5),
      osg::Vec3(-113.5, -2.5, -97.5)));

    input.push_back(Triangle(
      osg::Vec3(-32.5, 2.5, -97.5),
      osg::Vec3(-32.5, -2.5, -102.5),
      osg::Vec3(-32.5, -2.5, -97.5)));
    input.push_back(Triangle(
      osg::Vec3(-2.5, 118.5, -97.5),
      osg::Vec3(-2.5, 113.5, -102.5),
      osg::Vec3(-2.5, 113.5, -97.5)));
    input.push_back(Triangle(
      osg::Vec3(-2.5, 4.5, -97.5),
      osg::Vec3(-2.5, -0.5, -97.5),
      osg::Vec3(2.5, -0.5, -97.5)));
    input.push_back(Triangle(
      osg::Vec3(-2.5, -147.5, -102.5),
      osg::Vec3(2.5, -147.5, -102.5),
      osg::Vec3(2.5, -152.5, -102.5)));
    input.push_back(Triangle(
      osg::Vec3(-2.5, 96.5, -102.5),
      osg::Vec3(2.5, 96.5, -102.5),
      osg::Vec3(2.5, 91.5, -102.5)));

    input.push_back(Triangle(
      osg::Vec3(2.5, 77.5, -97.5),
      osg::Vec3(2.5, 77.5, -102.5),
      osg::Vec3(2.5, 82.5, -102.5)));
    input.push_back(Triangle(
      osg::Vec3(65.5, 2.5, -97.5),
      osg::Vec3(65.5, -2.5, -102.5),
      osg::Vec3(65.5, -2.5, -97.5)));
    input.push_back(Triangle(
      osg::Vec3(147.5, -2.5, -97.5),
      osg::Vec3(147.5, -2.5, -102.5),
      osg::Vec3(152.5, -2.5, -102.5)));
}

BOOST_AUTO_TEST_CASE(setOSGImagePixel_TestCase) {

    cv::Point ground_truth_points[] = {
                                  cv::Point(120,15),
                                  cv::Point(199,11),
                                  cv::Point(350,90),
                                  cv::Point(20,180),
                                  cv::Point(300,199),
                                  cv::Point(40,290),
                                  cv::Point(300,300),
                                  cv::Point(350,400),
                                  cv::Point(400,100) };

      cv::Point3f ground_truth_value[] = {
                                  cv::Point3f(1, 1, 1),
                                  cv::Point3f(1, 1, 1),
                                  cv::Point3f(0, 1, 1),
                                  cv::Point3f(0, 1, 0),
                                  cv::Point3f(0, 1, 0),
                                  cv::Point3f(1, 0, 0),
                                  cv::Point3f(0, 0, 1),
                                  cv::Point3f(1, 1, 0),
                                  cv::Point3f(0, 0, 0)};

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(500, 500, 3, GL_RGB, GL_FLOAT);

    for (unsigned int x = 100; x < 200 ; x++)
        for (unsigned int y = 10; y < 20 ; y++) {
            setOSGImagePixel(image, x, y, 0, 1.0f);
            setOSGImagePixel(image, x, y, 1, 1.0f);
            setOSGImagePixel(image, x, y, 2, 1.0f);
        }

    for (unsigned int x = 300; x < 400 ; x++)
        for (unsigned int y = 10; y < 100 ; y++) {
            setOSGImagePixel(image, x, y, 0, 1.0f);
            setOSGImagePixel(image, x, y, 1, 1.0f);
        }

    for (unsigned int x = 5; x < 490 ; x++)
        for (unsigned int y = 120; y < 200 ; y++) {
            setOSGImagePixel(image, x, y, 1, 1.0f);
        }

    for (unsigned int x = 10; x < 50 ; x++)
        for (unsigned int y = 210; y < 300 ; y++) {
            setOSGImagePixel(image, x, y, 2, 1.0f);
        }

    for (unsigned int x = 200; x < 490 ; x++)
        for (unsigned int y = 250; y < 350 ; y++) {
            setOSGImagePixel(image, x, y, 0, 1.0f);
        }

    for (unsigned int x = 50; x < 400 ; x++)
        for (unsigned int y = 380; y < 480 ; y++) {
            setOSGImagePixel(image, x, y, 1, 1.0f);
            setOSGImagePixel(image, x, y, 2, 1.0f);
        }

    cv::Mat cv_image = convertOSG2CV(image);
    for (unsigned int i = 0; i < 9; i++) {
        cv::Point point = ground_truth_points[i];
        cv::Point3f value( cv_image.at<cv::Vec3f>(point.x,point.y)[0],
                           cv_image.at<cv::Vec3f>(point.x,point.y)[1],
                           cv_image.at<cv::Vec3f>(point.x,point.y)[2]);
        BOOST_CHECK_EQUAL(value, ground_truth_value[i]);
    }
}

BOOST_AUTO_TEST_CASE(getAllDataAsVector_TestCase) {

      float ground_truth[] = {0, 5, 0, -5, -5, 0, 5, -5, 0,
                              0, -1.6666666, 0, 0, 0, 1};

      input_triangles.clear();
      triangleStructureDataSet(input_triangles);
      std::vector<float> output = input_triangles[0].getAllDataAsVector();
      for (unsigned int i = 0; i < output.size(); i++)
          BOOST_CHECK_CLOSE(ground_truth[i],
                            output[i],
                            0.001);
}

BOOST_AUTO_TEST_SUITE_END();
