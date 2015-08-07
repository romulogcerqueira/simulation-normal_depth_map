/*
 * NormalDepthMap_test.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: tiagotrocoli
 */

#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <iostream>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BOOST_TEST_MODULE "NormalDepthMap_test"
#include <boost/test/unit_test.hpp>

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(vizkit3d_NormalDepthMap)

//draw the scene with a small ball in the center with a big cube, cylinder and cone in back
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

void viewPointsFromScene1(std::vector<osg::Vec3d>* eyes, std::vector<osg::Vec3d>* centers, std::vector<osg::Vec3d>* ups) {

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

// reference points, and map values for each view in viewPointsFromScene1
void referencePointsFromScene1(std::vector<std::vector<cv::Point> >* setPoints, std::vector<std::vector<cv::Point3i> >* setValues) {

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

    // image points in view3
    points.push_back(cv::Point(75, 64));
    points.push_back(cv::Point(250, 251));
    points.push_back(cv::Point(410, 459));
    points.push_back(cv::Point(15, 485));
    points.push_back(cv::Point(461, 36));
    setPoints->push_back(points);

    std::vector<cv::Point3i> values;

    // pixel value from each point in image from view1
    values.push_back(cv::Point3i(878.431, 811.608, 0));
    values.push_back(cv::Point3i(0, 0, 0));
    values.push_back(cv::Point3i(909.804, 356.627, 0));
    values.push_back(cv::Point3i(254.902, 356.706, 0));
    values.push_back(cv::Point3i(250.98, 341.02, 0));
    values.push_back(cv::Point3i(0, 0, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view2
    values.push_back(cv::Point3i(0, 0, 0));
    values.push_back(cv::Point3i(917.647, 231.745, 0));
    values.push_back(cv::Point3i(925.49, 803.843, 0));
    values.push_back(cv::Point3i(86.2745, 788.157, 0));
    values.push_back(cv::Point3i(152.941, 196, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view3
    values.push_back(cv::Point3i(74.5098, 509.49, 0));
    values.push_back(cv::Point3i(823.529, 494.039, 0));
    values.push_back(cv::Point3i(0, 0, 0));
    values.push_back(cv::Point3i(0, 270.961, 0));
    values.push_back(cv::Point3i(780.392, 529.333, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view4
    values.push_back(cv::Point3i(956.863, 545.628, 0));
    values.push_back(cv::Point3i(1000, 545.628, 0));
    values.push_back(cv::Point3i(956.863, 545.628, 0));
    values.push_back(cv::Point3i(0, 0, 0));
    values.push_back(cv::Point3i(0, 0, 0));
    setValues->push_back(values);
    values.clear();
}

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {

    std::vector<osg::Vec3d> eyes, centers, ups;
    std::vector<std::vector<cv::Point> > setPoints;
    std::vector<std::vector<cv::Point3i> > setValues;
    referencePointsFromScene1(&setPoints, &setValues);

    float maxRange = 50;
    uint width = 500, height = 500;
    NormalDepthMap normalDepthMap(maxRange);
    ImageViewerCaptureTool capture(width, height);
    capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

    osg::ref_ptr<osg::Group> root = new osg::Group();
    viewPointsFromScene1(&eyes, &centers, &ups);
    makeSimpleScene1(root);
    normalDepthMap.addNodeChild(root);

    uint precision = 1000;

    for (uint i = 0; i < eyes.size(); ++i) {
        capture.setCameraPosition(eyes[i], centers[i], ups[i]);

        normalDepthMap.setDrawNormal(true);
        normalDepthMap.setDrawDepth(true);
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
        cv::Mat3f cvImage(osgImage->t(), osgImage->s());
        cvImage.data = osgImage->data();
        cvImage = cvImage.clone();
        cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(cvImage, cvImage, 0);

//         get only normal map
        normalDepthMap.setDrawDepth(false);
        osg::ref_ptr<osg::Image> osgImageNormalMap = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
        cv::Mat3f cvImageNormalMap(osgImage->t(), osgImage->s());
        cvImageNormalMap.data = osgImageNormalMap->data();
        cvImageNormalMap = cvImageNormalMap.clone();
        cv::cvtColor(cvImageNormalMap, cvImageNormalMap, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(cvImageNormalMap, cvImageNormalMap, 0);

//         get only half range depth map;
        normalDepthMap.setDrawDepth(true);
        normalDepthMap.setDrawNormal(false);
        osg::ref_ptr<osg::Image> osgImageDepthMap = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
        cv::Mat3f cvImageDepthMap(osgImage->t(), osgImage->s());
        cvImageDepthMap.data = osgImageDepthMap->data();
        cv::cvtColor(cvImageDepthMap, cvImageDepthMap, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(cvImageDepthMap, cvImageDepthMap, 0);

        for (uint j = 0; j < setPoints[i].size(); ++j) {
            cv::Point p = setPoints[i][j];

            cv::Point3i imgValue(cvImage[p.y][p.x][0] * precision, cvImage[p.y][p.x][1] * precision, cvImage[p.y][p.x][2] * precision);
            cv::Point3i imgValueNormalMap(cvImageNormalMap[p.y][p.x][0] * precision, cvImageNormalMap[p.y][p.x][1] * precision, cvImageNormalMap[p.y][p.x][2] * precision);
            cv::Point3i imgValueDepthMap(cvImageDepthMap[p.y][p.x][0] * precision, cvImageDepthMap[p.y][p.x][1] * precision, cvImageDepthMap[p.y][p.x][2] * precision);

            BOOST_CHECK_EQUAL(imgValue, setValues[i][j]);
            BOOST_CHECK_EQUAL(imgValueNormalMap, cv::Point3i(setValues[i][j].x, 0, 0));
            BOOST_CHECK_EQUAL(imgValueDepthMap, cv::Point3i(0, setValues[i][j].y, 0));

//            std::cout << "INDEX" << i << " NORMAL=" << cvImage[p.y][p.x][0] << " DIST=" << cvImage[p.y][p.x][1] << std::endl;
//            cv::circle(cvImage, p, 2, cv::Scalar(0, 0, 255), -1);
//            cv::imshow("Test Image", cvImage);
//            cv::waitKey();
        }
    }

}

void checkDepthValueRadialVariation(cv::Mat3f image, uint id) {

    static const int groudTruth0[] = { 4549, 4666, 4745, 4823, 4901, 4941, 5019, 5058, 5098, 5098, 5098, 5137, 5098, 5098, 5058, 5019, 4980, 4941, 4862, 4784, 4705, 4627 };
    static const int groudTruth1[] = { 274, 980, 1686, 2352, 2980, 3568, 4078, 4509, 4862, 5058, 5137, 5058, 4823, 4509, 4039, 3529, 2941, 2313, 1647, 941 };
    static const int groudTruth2[] = { 1921, 2431, 2901, 3333, 3764, 4156, 4470, 4745, 4941, 5058, 5137, 5098, 4941, 4745, 4509, 4156, 3803, 3372, 2941, 2431, 1921 };

    std::vector<std::vector<int> > groundTruthVector(3);
    groundTruthVector[0] = std::vector<int>(groudTruth0, groudTruth0 + sizeof(groudTruth0) / sizeof(int));
    groundTruthVector[1] = std::vector<int>(groudTruth1, groudTruth1 + sizeof(groudTruth1) / sizeof(int));
    groundTruthVector[2] = std::vector<int>(groudTruth2, groudTruth2 + sizeof(groudTruth2) / sizeof(int));

    cv::Point centerPoint(image.size().width / 2, image.size().height / 2);
    std::vector<int> histSizes;
    for (int i = 0; i < image.size().width; i = i + image.size().width * 0.05)
        histSizes.push_back((uint) (image[centerPoint.y][i][1] * 10000));

    BOOST_CHECK_EQUAL_COLLECTIONS(histSizes.begin(), histSizes.end(), groundTruthVector[id].begin(), groundTruthVector[id].end());
}

BOOST_AUTO_TEST_CASE(depthValueRadialVariation_testCase) {

    osg::ref_ptr<osg::Geode> scene = new osg::Geode();
    osg::ref_ptr<osg::Shape> box;
    uint numberSphere = 100;
    double multi = 2;
    double boxSize = 5;
    double distance = 100;
    double maxRange = 200;
    for (uint i = 0; i < numberSphere; ++i) {
        box = new osg::Box(osg::Vec3(i * multi, 0, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(i * -multi, 0, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(0, i * -multi, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(0, i * multi, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
    }
    NormalDepthMap normalDepthMap(maxRange);
    normalDepthMap.setDrawNormal(false);
    normalDepthMap.addNodeChild(scene);

    uint sizeVector = 3;
    double fovys[] = { 150, 120, 20 };
    double fovxs[] = { 20, 120, 150 };
    uint heightSize[] = { 500, 500, 500 };

    for (uint j = 0; j < sizeVector; ++j) {
        ImageViewerCaptureTool capture(fovys[j], fovxs[j], heightSize[j]);
        capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
        cv::Mat3f cvImage(osgImage->t(), osgImage->s());
        cvImage.data = osgImage->data();
        cvImage = cvImage.clone();
        cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(cvImage, cvImage, 0);
        checkDepthValueRadialVariation(cvImage, j);
    }
}

BOOST_AUTO_TEST_SUITE_END();

