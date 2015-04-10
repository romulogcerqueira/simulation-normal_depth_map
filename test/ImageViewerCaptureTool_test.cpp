#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>

#define BOOST_TEST_MODULE "ImageViewerCaptureTool_test"
#include <boost/test/unit_test.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <osg/Geode>
#include <osg/ShapeDrawable>

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(vizkit3d_ImageViewerCaptureTool)

// dataset function
void viewPointsFromScene(osg::ref_ptr<osg::Geode> scene, std::vector<osg::Vec3d>* eyes, std::vector<osg::Vec3d>* centers, std::vector<osg::Vec3d>* ups, std::vector<osg::Vec4d>* backgrounds,
        std::vector<std::vector<cv::Point> > *setPoints, std::vector<std::vector<cv::Point3i> > *setValues) {

    // draw a small Sphere
    osg::ref_ptr<osg::Shape> sphere = new osg::Sphere(osg::Vec3(), 1);
    scene->addDrawable(new osg::ShapeDrawable(sphere));

    std::vector<cv::Point> points;
    std::vector<cv::Point3i> values;

    // set view1
    eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
    centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
    ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));
    backgrounds->push_back(osg::Vec4d(0.11, 0.5, 0.76, 0));
    points.push_back(cv::Point(100, 100));
    points.push_back(cv::Point(400, 100));
    points.push_back(cv::Point(100, 400));
    points.push_back(cv::Point(400, 400));
    values.push_back(cv::Point3i(760, 498, 109));
    values.push_back(cv::Point3i(760, 498, 109));
    values.push_back(cv::Point3i(866, 866, 866));
    values.push_back(cv::Point3i(760, 498, 109));

    setPoints->push_back(points);
    setValues->push_back(values);
    points.clear();
    values.clear();

    // set view2
    eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
    centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
    ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));
    backgrounds->push_back(osg::Vec4d(0.3, 0.8, 0.55, 0));
    points.push_back(cv::Point(100, 100));
    points.push_back(cv::Point(400, 100));
    points.push_back(cv::Point(350, 100));
    points.push_back(cv::Point(400, 150));
    values.push_back(cv::Point3i(549, 800, 298));
    values.push_back(cv::Point3f(843, 843, 843));
    values.push_back(cv::Point3f(854, 854, 854));
    values.push_back(cv::Point3f(792, 792, 792));

    setPoints->push_back(points);
    setValues->push_back(values);
    points.clear();
    values.clear();

}

BOOST_AUTO_TEST_CASE(ImageViewerCaptureTool_TestCase) {

    osg::ref_ptr<osg::Geode> scene = new osg::Geode();
    std::vector<osg::Vec3d> eyes, centers, ups;
    std::vector<osg::Vec4d> backgrounds;
    std::vector<std::vector<cv::Point> > setPoints;
    std::vector<std::vector<cv::Point3i> > setValues;

    viewPointsFromScene(scene, &eyes, &centers, &ups, &backgrounds, &setPoints, &setValues);
    ImageViewerCaptureTool capture(500, 500);
    uint precision = 1000;

    for (uint i = 0; i < eyes.size(); ++i) {
        capture.setBackgroundColor(backgrounds[i]);
        capture.setCameraPosition(eyes[i], centers[i], ups[i]);
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(scene);
        cv::Mat3f img, cvImgBuf(osgImage->t(), osgImage->s());
        cvImgBuf.data = osgImage->data();
        cv::cvtColor(cvImgBuf, img, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(img, img, 0);
        for (uint j = 0; j < setPoints[i].size(); ++j) {
            cv::Point p = setPoints[i][j];
            cv::Point3i imgValue(img[p.y][p.x][0] * precision, img[p.y][p.x][1] * precision, img[p.y][p.x][2] * precision);
            BOOST_CHECK_EQUAL(imgValue, setValues[i][j]);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END();
