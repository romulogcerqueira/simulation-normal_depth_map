#define BOOST_TEST_MODULE "BumpMapping_test"
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>

// OpenSceneGraph includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/Image>
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

const int TEXTURE_UNIT_DIFFUSE = 0;
const int TEXTURE_UNIT_NORMAL = 1;
const int TEXTURE_UNIT_SPECULAR = 2;

using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(BumpMapping)

// check if two matrixes are equals
bool are_equals (const cv::Mat& image1, const cv::Mat& image2) {
    cv::Mat diff = image1 != image2;
    return (cv::countNonZero(diff) == 0);
}

// add a simple sphere to scene
void addSimpleObject(osg::ref_ptr<osg::Group> root) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,-7.5), 3)));
    root->addChild(geode);
    root->getChild(0)->asGeode()->addDrawable(geode->getDrawable(0));
}

// define texture attributes
osg::ref_ptr<osg::StateSet> insertBumpMapTexture(osg::ref_ptr<osg::Image> diffuseImage, osg::ref_ptr<osg::Image> normalImage, osg::ref_ptr<osg::Image> specularImage) {
    osg::ref_ptr<osg::Texture2D> diffuse = new osg::Texture2D();
    osg::ref_ptr<osg::Texture2D> normal = new osg::Texture2D();
    osg::ref_ptr<osg::Texture2D> specular = new osg::Texture2D();

    diffuse->setImage(diffuseImage);
    diffuse->setDataVariance(osg::Object::DYNAMIC);
    diffuse->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    diffuse->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    diffuse->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    diffuse->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    diffuse->setResizeNonPowerOfTwoHint(false);
    diffuse->setMaxAnisotropy(8.0f);

    normal->setImage(normalImage);
    normal->setDataVariance(osg::Object::DYNAMIC);
    normal->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    normal->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    normal->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    normal->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    normal->setResizeNonPowerOfTwoHint(false);
    normal->setMaxAnisotropy(8.0f);

    specular->setImage(specularImage);
    specular->setDataVariance(osg::Object::DYNAMIC);
    specular->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    specular->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    specular->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    specular->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    specular->setResizeNonPowerOfTwoHint(false);
    specular->setMaxAnisotropy(8.0f);

    osg::StateSet* bumpState = new osg::StateSet();
    bumpState->setTextureAttributeAndModes(TEXTURE_UNIT_DIFFUSE, diffuse, osg::StateAttribute::ON);
    bumpState->setTextureAttributeAndModes(TEXTURE_UNIT_NORMAL, normal, osg::StateAttribute::ON);
    bumpState->setTextureAttributeAndModes(TEXTURE_UNIT_SPECULAR, specular, osg::StateAttribute::ON);
    return bumpState;
}

void loadTextures(osg::ref_ptr<osg::Group> root) {
    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));

    // load texture files
    osg::ref_ptr<osg::Image> diffuseImage = osgDB::readImageFile(current_path + "/textures/red_texture_d.jpg");
    osg::ref_ptr<osg::Image> normalImage = osgDB::readImageFile(current_path + "/textures/red_texture_n.jpg");
    osg::ref_ptr<osg::Image> specularImage = osgDB::readImageFile(current_path + "/textures/red_texture_s.jpg");
    BOOST_CHECK( (!diffuseImage || !normalImage || !specularImage) == false );

    // texture properties
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->setStateSet(insertBumpMapTexture(diffuseImage, normalImage, specularImage));
    root->addChild(geode);
}

// create simple scene without texture
osg::ref_ptr<osg::Group> createSimpleScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addSimpleObject(root);
    return root;
}

// create scene with bump mapping
osg::ref_ptr<osg::Group> createBumpMapScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("diffuseTexture", TEXTURE_UNIT_DIFFUSE));
    stateset->addUniform(new osg::Uniform("normalTexture", TEXTURE_UNIT_NORMAL));
    stateset->addUniform(new osg::Uniform("specularTexture", TEXTURE_UNIT_SPECULAR));
    stateset->setDataVariance(osg::Object::STATIC);
    root->setStateSet(stateset);

    loadTextures(root);
    addSimpleObject(root);
    return root;
}

// compute the normal map for a osg scene
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

void plotSonarTest(cv::Mat3f image, double maxRange, double maxAngleX) {

    cv::Mat1b imagePlot = cv::Mat1b::zeros(500, 500);
    cv::Point2f centerPlot(imagePlot.cols / 2, 0);
    double factor = 500 / maxRange;
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

    cv::imshow("Sonar Plot Test", imagePlotMap);
    cv::imshow("Normal Depth Map", image);
    cv::waitKey();
}


BOOST_AUTO_TEST_CASE(differentNormalMaps_TestCase) {
    float maxRange = 10.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> simpleRoot = createSimpleScene();
    osg::ref_ptr<osg::Group> bumpRoot = createBumpMapScene();

    cv::Mat cvSimple = computeNormalDepthMap(simpleRoot, maxRange, fovX, fovY);
    cv::Mat cvBump = computeNormalDepthMap(bumpRoot, maxRange, fovX, fovY);

    std::vector<cv::Mat> simpleChannels, bumpChannels;
    cv::split(cvSimple, simpleChannels);
    cv::split(cvBump, bumpChannels);

    // assert that the normal matrixes are different
    BOOST_CHECK(are_equals(simpleChannels[0], bumpChannels[0]) == false);

    // assert that the depth matrixes are equals
    BOOST_CHECK(are_equals(simpleChannels[1], bumpChannels[1]) == true);

    // plot sonar sample output
    plotSonarTest(cvSimple, maxRange, fovX * 0.5);
    plotSonarTest(cvBump, maxRange, fovX * 0.5);
}

BOOST_AUTO_TEST_SUITE_END()//
