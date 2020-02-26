// OSG includes
#include <osgViewer/Viewer>
#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/BoundingBox>

// C++ includes
#include <iostream>
#include <vector>
#include <numeric>

// Rock includes
#include <normal_depth_map/Tools.hpp>
#include "TestHelper.hpp"

#define BOOST_TEST_MODULE "Reverberation_test"
#include <boost/test/unit_test.hpp>

#define myrand ((float)(random()) / (float)(RAND_MAX))

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(Validation)

struct MyTriangle
{
    std::vector<osg::Vec3f> data;

    MyTriangle()
        : data(5, osg::Vec3f(0, 0, 0)){};

    MyTriangle(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
        : data(5, osg::Vec3f(0, 0, 0))
    {
        setTriangle(v1, v2, v3);
    };

    void setTriangle(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
    {
        data[0] = v1;                       // vertex 1
        data[1] = v2;                       // vertex 2
        data[2] = v3;                       // vertex 3
        data[3] = (v1 + v2 + v3) / 3;       // centroid
        data[4] = (v2 - v1)^(v3 - v1);
        data[4].normalize();                // surface normal
    };

    // get the triangle data as vector of float
    std::vector<float> getAllDataAsVector()
    {
        float *array = &data[0].x();
        uint arraySize = data.size() * data[0].num_components;
        std::vector<float> output(array, array + arraySize);
        return output;
    }
};

struct MyBoundingBox
{
    std::vector<osg::Vec3f> data;

    MyBoundingBox()
        : data(2, osg::Vec3f(0, 0, 0)){};

    MyBoundingBox(osg::Vec3f min, osg::Vec3f max)
        : data(2)
    {
        data[0] = min;
        data[1] = max;
    };

    // get the triangle data as vector of float
    std::vector<float> getAllDataAsVector()
    {
        float *array = &data[0].x();
        uint arraySize = data.size() * data[0].num_components;
        std::vector<float> output(array, array + arraySize);
        return output;
    }
};

class MyTrianglesVisitor : public osg::NodeVisitor
{
  protected:
    struct WorldTriangle
    {
        std::vector<MyTriangle> triangles;
        osg::Matrixd local2world;

        inline void operator()(const osg::Vec3f &v1,
                               const osg::Vec3f &v2,
                               const osg::Vec3f &v3,
                               bool treatVertexDataAsTemporary)
        {
            // transform vertice coordinates to world coordinates
            osg::Vec3f v1_w = v1 * local2world;
            osg::Vec3f v2_w = v2 * local2world;
            osg::Vec3f v3_w = v3 * local2world;
            triangles.push_back(MyTriangle(v1_w, v2_w, v3_w));
        };
    };
    osg::TriangleFunctor<WorldTriangle> tf;
    std::vector<uint> trianglesRef;
    std::vector<MyBoundingBox> bboxes;

  public:
    MyTrianglesVisitor()
    {
        setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        trianglesRef.push_back(0);
    };

    void apply(osg::Geode &geode)
    {
        // local to world matrix
        tf.local2world = osg::computeLocalToWorld(this->getNodePath());

        for (size_t idx = 0; idx < geode.getNumDrawables(); ++idx)
        {
            // triangles
            geode.getDrawable(idx)->accept(tf);
            trianglesRef.push_back(tf.triangles.size());

            // bounding box
            osg::BoundingBox bb = geode.getDrawable(idx)->getBound();
            MyBoundingBox bb_w(bb._min * tf.local2world, bb._max * tf.local2world);
            bboxes.push_back(bb_w);
        }
    }

    std::vector<MyTriangle> getTriangles() { return tf.triangles; };
    std::vector<uint> getTrianglesRef() { return trianglesRef; };
    std::vector<MyBoundingBox> getBoundingBoxes() { return bboxes; };
};

// print triangles data
template <typename T>
void printMyData(std::vector<T> &myData)
{
    for (size_t i = 0; i < myData.size(); i++)
    {
        std::cout << "\nData " << i << "\n------------" << std::endl;
        T md = myData[i];

        for (size_t j = 0; j < md.data.size(); j++)
        {
            std::cout << md.data[j].x() << ", "
                      << md.data[j].y() << ", "
                      << md.data[j].z() << std::endl;
        }
    }
}

// draw the references for bounding boxes
void drawBBoxesReferences(osg::ref_ptr<osg::Group> root, std::vector<MyBoundingBox> bboxes)
{
    for (size_t i = 0; i < bboxes.size(); i++)
    {
        osg::TessellationHints *hints = new osg::TessellationHints();
        hints->setDetailRatio(2.0f);

        osg::Geode *geode = new osg::Geode();
        osg::ShapeDrawable *shape = new osg::ShapeDrawable();

        shape = new osg::ShapeDrawable(new osg::Sphere(bboxes[i].data[0], 0.2), hints);
        shape->setColor(osg::Vec4(1.0f, 0.f, 0.f, 1.0f));
        geode->addDrawable(shape);

        shape = new osg::ShapeDrawable(new osg::Sphere(bboxes[i].data[1], 0.2), hints);
        shape->setColor(osg::Vec4(1.0f, 0.f, 0.f, 1.0f));
        geode->addDrawable(shape);

        root->addChild(geode);
    }
}

// convert triangles into texture (to be read by shader)
void triangles2texture(
    std::vector<MyTriangle> triangles,
    std::vector<uint> trianglesRef,
    std::vector<MyBoundingBox> bboxes,
    osg::ref_ptr<osg::Texture2D> &texture)
{
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage((triangles.size() + trianglesRef.size() + bboxes.size()),
                         triangles[0].getAllDataAsVector().size(),
                         1,
                         GL_RED,
                         GL_FLOAT);
    image->setInternalTextureFormat(GL_R32F);

    // set triangles data into texture
    for (size_t j = 0; j < triangles.size(); j++)
    {
        std::vector<float> data = triangles[j].getAllDataAsVector();
        for (size_t i = 0; i < data.size(); i++)
            setOSGImagePixel(image, i, j, 0, data[i]);
    }

    // set triangles reference into texture
    for (size_t i = 0; i < trianglesRef.size(); i++)
    {
        size_t idx = triangles.size() + i;
        setOSGImagePixel(image, 0, idx, 0, (float)trianglesRef[i]);
    }

    // set bounding boxes into texture
    for(size_t j = 0; j < bboxes.size(); j++)
    {
        std::vector<float> data = bboxes[j].getAllDataAsVector();
        for (size_t i = 0; i < data.size(); i++)
            setOSGImagePixel(image, i, j, 0, data[i]);
    }

    // set texture
    texture = new osg::Texture2D;
    texture->setTextureSize(image->s(), image->t());
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setUnRefImageDataAfterApply(true);
    texture->setImage(image);
}

BOOST_AUTO_TEST_CASE(reverberation_testCase)
{
    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> scene = new osg::Group();
    makeDemoScene2(scene);

    // triangles and bounding boxes
    MyTrianglesVisitor visitor;
    scene->accept(visitor);

    std::vector<MyTriangle> triangles = visitor.getTriangles();
    std::vector<uint> trianglesRef = visitor.getTrianglesRef();
    std::vector<MyBoundingBox> bboxes = visitor.getBoundingBoxes();

    std::cout << "\nTriangles Reference" << std::endl;
    for (size_t idx = 0; idx < trianglesRef.size(); idx++)
        std::cout << trianglesRef[idx] << std::endl;

    // print triangles
    // std::cout << "\m======== TRIANGLES ===========" << std::endl;
    // printMyData(triangles);

    // print bounding boxes
    std::cout << "\n======== BOUNDING BOXES ======" << std::endl;
    printMyData(bboxes);


    std::vector<float> bbdata;
    for(size_t i = 0; i < bboxes.size(); i++)
    {
        std::vector<float> tmp = bboxes[i].getAllDataAsVector();
        for(size_t j = 0; j < tmp.size(); j++)
            bbdata.push_back(tmp[j]);
    }

    std::cout << "\ndata in float vector" << std::endl;
    for(size_t i = 0; i < bbdata.size(); i++)
    {
        std::cout << bbdata[i] << std::endl;
    }

    osg::ref_ptr<osg::Texture2D> trianglesTexture;
    triangles2texture(triangles, trianglesRef, bboxes, trianglesTexture);

    // draw references for bounding boxes
    drawBBoxesReferences(scene, bboxes);

    // sonar parameters
    float maxRange = 15;   // 15 meters
    float fovX = M_PI / 4; // 45 degrees
    float fovY = M_PI / 4; // 45 degrees

    // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromDemoScene2(&eyes, &centers, &ups);

    // compute and display the final shader and sonar images
    for (unsigned i = 0; i < eyes.size(); ++i)
    {
        cv::Mat shaderImg = computeNormalDepthMap(scene, maxRange, fovX, fovY, 0, eyes[i], centers[i], ups[i]);
        cv::imshow("shaderImg", shaderImg);
        cv::waitKey();
    }

    // view 3D scene using osgviewer
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(0, 0, 600, 600);
    viewer.setSceneData(scene);
    viewer.run();
}

BOOST_AUTO_TEST_SUITE_END();