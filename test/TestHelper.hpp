// OpenCV includes
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OSG includes
#include <osg/Group>
#include <osg/Image>

namespace test_helper {
    cv::Mat drawSonarImage(cv::Mat3f image, double maxRange, double maxAngleX);
    cv::Mat computeNormalDepthMap(osg::ref_ptr<osg::Group> root,
                                  float maxRange,
                                  float fovX,
                                  float fovY,
                                  double attenuationCoeff = 0,
                                  osg::Vec3d eye    = osg::Vec3d(0,0,0),
                                  osg::Vec3d center = osg::Vec3d(0,0,-1),
                                  osg::Vec3d up     = osg::Vec3d(0,1,0),
                                  uint height = 500
                                );
    void roundMat(cv::Mat& roi, int precision);
    bool areEqualImages(const cv::Mat& image1, const cv::Mat& image2);

    template <typename T>
    static bool areEqualVectors(std::vector<T> a, std::vector<T> b) {
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        return (a == b);
    }

    // draw the scene with a small ball in the center with a big cube, cylinder and cone in back
    void makeDemoScene(osg::ref_ptr<osg::Group> root);
    void makeDemoScene2(osg::ref_ptr<osg::Group> root);

    // define different point of views of the same scene
    void viewPointsFromDemoScene(std::vector<osg::Vec3d> *eyes,
                                 std::vector<osg::Vec3d> *centers,
                                 std::vector<osg::Vec3d> *ups);

    // define different point of views of the same scene
    void viewPointsFromDemoScene2(std::vector<osg::Vec3d> *eyes,
                                  std::vector<osg::Vec3d> *centers,
                                  std::vector<osg::Vec3d> *ups);

    // convert opencv to osg images
    osg::ref_ptr<osg::Image> convertCV2OSG(const cv::Mat& cv_image);

    // convert osg to opencv images
    cv::Mat convertOSG2CV(const osg::ref_ptr<osg::Image>& osgImage);
}
