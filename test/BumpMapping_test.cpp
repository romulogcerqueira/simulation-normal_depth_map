#define BOOST_TEST_MODULE "BumpMapping_test"
#include <boost/test/unit_test.hpp>

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

using namespace vizkit3d_normal_depth_map;

enum TextureUnitTypes {
    TEXTURE_UNIT_DIFFUSE,
    TEXTURE_UNIT_NORMAL
};

enum TextureImages {
    TEXTURE_CONCRETE,
    TEXTURE_GRAY,
    TEXTURE_ROCKS
};

BOOST_AUTO_TEST_SUITE(BumpMapping)

// check if two matrixes are equals
bool are_equals (const cv::Mat& image1, const cv::Mat& image2) {
    cv::Mat diff = image1 != image2;
    return (cv::countNonZero(diff) == 0);
}

// add one object to scene (sphere)
void addSimpleObject(osg::ref_ptr<osg::Group> root) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,-14), 5)));
    root->addChild(geode);
    root->getChild(0)->asGeode()->addDrawable(geode->getDrawable(0));
}

// add two objects to scene (sphere and box)
void addMultiObject(osg::ref_ptr<osg::Group> root) {
    osg::ref_ptr<osg::Geode> sphere = new osg::Geode();
    sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-2, -2, -10), 1.25)));
    root->addChild(sphere);
    root->getChild(0)->asGeode()->addDrawable(sphere->getDrawable(0));

    osg::ref_ptr<osg::Geode> box = new osg::Geode();
    box->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(2, 2, -20), 7.5)));
    root->addChild(box);
    root->getChild(1)->asGeode()->addDrawable(box->getDrawable(0));
}

// define texture attributes
osg::ref_ptr<osg::StateSet> insertBumpMapTexture(osg::ref_ptr<osg::Image> diffuseImage, osg::ref_ptr<osg::Image> normalImage) {
    osg::ref_ptr<osg::Texture2D> diffuse = new osg::Texture2D();
    osg::ref_ptr<osg::Texture2D> normal = new osg::Texture2D();

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

    osg::ref_ptr<osg::StateSet> bumpState = new osg::StateSet();
    bumpState->setTextureAttributeAndModes(TEXTURE_UNIT_DIFFUSE, diffuse, osg::StateAttribute::ON);
    bumpState->setTextureAttributeAndModes(TEXTURE_UNIT_NORMAL, normal, osg::StateAttribute::ON);
    return bumpState;
}

// get texture files
void loadTextures(osg::ref_ptr<osg::Group> root, TextureImages textureId) {
    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));

    // load texture files
    std::string texture_type;
    switch(textureId) {
        case TEXTURE_CONCRETE:
            texture_type = "concrete_texture";
            break;
        case TEXTURE_GRAY:
            texture_type = "gray_texture";
            break;
        case TEXTURE_ROCKS:
            texture_type = "rocks_texture";
            break;
        default:
            throw std::invalid_argument("Texture image parameter does not match a known enum value");
    }

    osg::ref_ptr<osg::Image> diffuseImage = osgDB::readImageFile(current_path + "/textures/" + texture_type + "_d.jpg");
    osg::ref_ptr<osg::Image> normalImage = osgDB::readImageFile(current_path + "/textures/" + texture_type + "_n.jpg");
    BOOST_CHECK( (!diffuseImage || !normalImage) == false );

    // texture properties
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->setStateSet(insertBumpMapTexture(diffuseImage, normalImage));
    root->addChild(geode);
}

// create simple scene without texture
osg::ref_ptr<osg::Group> createSimpleScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addSimpleObject(root);
    return root;
}

// create scene with bump mapping and one object
osg::ref_ptr<osg::Group> createBumpMapSimpleScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("diffuseTexture", TEXTURE_UNIT_DIFFUSE));
    stateset->addUniform(new osg::Uniform("normalTexture", TEXTURE_UNIT_NORMAL));
    stateset->setDataVariance(osg::Object::STATIC);
    root->setStateSet(stateset);

    loadTextures(root, TEXTURE_GRAY);
    addSimpleObject(root);
    return root;
}

// create scene with bump mapping and multiple objects
osg::ref_ptr<osg::Group> createBumpMapMultiScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("diffuseTexture", TEXTURE_UNIT_DIFFUSE));
    stateset->addUniform(new osg::Uniform("normalTexture", TEXTURE_UNIT_NORMAL));
    stateset->setDataVariance(osg::Object::STATIC);
    root->setStateSet(stateset);

    loadTextures(root, TEXTURE_CONCRETE);
    loadTextures(root, TEXTURE_ROCKS);
    addMultiObject(root);
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

// simple polar plot
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

cv::Mat getNormalGroundTruth() {
    cv::Mat normalGroundTruth = cv::Mat::zeros(cv::Size(5,5), CV_32FC1);

    normalGroundTruth.at<float>(0,0) = 0.02352;
    normalGroundTruth.at<float>(1,0) = 0.03921;
    normalGroundTruth.at<float>(2,0) = 0.15294;
    normalGroundTruth.at<float>(3,0) = 0.24313;
    normalGroundTruth.at<float>(4,0) = 0.30196;

    normalGroundTruth.at<float>(0,1) = 0.02745;
    normalGroundTruth.at<float>(1,1) = 0.11372;
    normalGroundTruth.at<float>(2,1) = 0.20784;
    normalGroundTruth.at<float>(3,1) = 0.3098;
    normalGroundTruth.at<float>(4,1) = 0.35686;

    normalGroundTruth.at<float>(0,2) = 0.12156;
    normalGroundTruth.at<float>(1,2) = 0.1647;
    normalGroundTruth.at<float>(2,2) = 0.26274;
    normalGroundTruth.at<float>(3,2) = 0.37254;
    normalGroundTruth.at<float>(4,2) = 0.38823;

    normalGroundTruth.at<float>(0,3) = 0.14901;
    normalGroundTruth.at<float>(1,3) = 0.16862;
    normalGroundTruth.at<float>(2,3) = 0.31372;
    normalGroundTruth.at<float>(3,3) = 0.36862;
    normalGroundTruth.at<float>(4,3) = 0.37254;

    normalGroundTruth.at<float>(0,4) = 0.1647;
    normalGroundTruth.at<float>(1,4) = 0.21568;
    normalGroundTruth.at<float>(2,4) = 0.25098;
    normalGroundTruth.at<float>(3,4) = 0.27843;
    normalGroundTruth.at<float>(4,4) = 0.28627;

    return normalGroundTruth;
}

float roundtoPrecision(float value, int precision) {
    float output = (float) ((int) (value * pow(10, precision)) / pow(10,precision));
    return output;
}


BOOST_AUTO_TEST_CASE(differentNormalMaps_TestCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> simpleRoot = createSimpleScene();
    osg::ref_ptr<osg::Group> bumpRoot = createBumpMapSimpleScene();

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

BOOST_AUTO_TEST_CASE(multiTextureScene_TestCase) {
    float maxRange = 25.0f;
    float fovX = M_PI / 4;  // 45 degrees
    float fovY = M_PI / 4;  // 45 degrees

    osg::ref_ptr<osg::Group> bumpRoot = createBumpMapMultiScene();
    cv::Mat cvBump = computeNormalDepthMap(bumpRoot, maxRange, fovX, fovY);

    // plot sonar sample output
    plotSonarTest(cvBump, maxRange, fovX * 0.5);
}

BOOST_AUTO_TEST_CASE(pixelValidation_TestCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> bumpRoot = createBumpMapSimpleScene();
    cv::Mat cvBump = computeNormalDepthMap(bumpRoot, maxRange, fovX, fovY);

    cv::Mat normalRoi;
    cv::extractChannel(cvBump(cv::Rect(160,175,5,5)), normalRoi, 0);
    for (size_t x = 0; x < normalRoi.cols; x++) {
        for (size_t y = 0; y < normalRoi.rows; y++) {
            float value = roundtoPrecision(normalRoi.at<float>(y, x), 5);
            normalRoi.at<float>(y,x) = value;
        }
    }

    cv::Mat normalGroundTruth = getNormalGroundTruth();

    BOOST_CHECK(are_equals(normalRoi, normalGroundTruth) == true);
}

BOOST_AUTO_TEST_SUITE_END()
