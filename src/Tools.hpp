#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_

// C++ includes
#include <vector>
#include <iostream>

// OSG includes
#include <osg/Node>
#include <osg/Geode>
#include <osg/ref_ptr>
#include <osg/TriangleFunctor>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgUtil/TangentSpaceGenerator>

namespace normal_depth_map {

    /**
     * @brief compute Underwater Signal Attenuation coefficient
     *
     *  This method is based on paper "A simplified formula for viscous and
     *  chemical absorption in sea water". The method computes the attenuation
     *  coefficient that will be used on shader normal intensite return.
     *
     *  @param double frequency: sound frequency in kHz.
     *  @param double temperature: water temperature in Celsius degrees.
     *  @param double depth: distance from water surface in meters.
     *  @param double salinity: amount of salt dissolved in a body of water in ppt.
     *  @param double acidity: pH water value.
     *
     *  @return double coefficient attenuation value
     */

    double underwaterSignalAttenuation( const double frequency,
                                        const double temperature,
                                        const double depth,
                                        const double salinity,
                                        const double acidity);

    /**
     * @brief
     *
     */
    struct Triangle
    {
        std::vector<osg::Vec3f> data;

        Triangle()
            : data(5, osg::Vec3f(0, 0, 0)){};

        Triangle(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
            : data(5, osg::Vec3f(0, 0, 0))
        {
            setTriangle(v1, v2, v3);
        };

        void setTriangle(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
        {
            data[0] = v1;                   // vertex 1
            data[1] = v2;                   // vertex 2
            data[2] = v3;                   // vertex 3
            data[3] = (v1 + v2 + v3) / 3;   // centroid
            data[4] = (v2 - v1)^(v3 - v1);
            data[4].normalize();            // surface normal
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

    /**
     * @brief
     *
     */
    struct BoundingBox
    {
        std::vector<osg::Vec3f> data;

        BoundingBox()
            : data(2, osg::Vec3f(0, 0, 0)){};

        BoundingBox(osg::Vec3f min, osg::Vec3f max)
            : data(2, osg::Vec3f(0, 0, 0))
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

    /**
     * @brief
     *
     */
    class TrianglesVisitor : public osg::NodeVisitor
    {
      protected:
        struct WorldTriangle
        {
            std::vector<Triangle> triangles;
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
                triangles.push_back(Triangle(v1_w, v2_w, v3_w));
            };
        };
        osg::TriangleFunctor<WorldTriangle> tf;
        std::vector<uint> trianglesRef;
        std::vector<BoundingBox> bboxes;

      public:
        TrianglesVisitor()
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

                // bounding boxes
                osg::BoundingBox bb = geode.getDrawable(idx)->getBound();
                BoundingBox bb_w(bb._min * tf.local2world, bb._max * tf.local2world);
                bboxes.push_back(bb_w);
            }
        }

        std::vector<Triangle> getTriangles() { return tf.triangles; };
        std::vector<uint> getTrianglesRef() { return trianglesRef; };
        std::vector<BoundingBox> getBoundingBoxes() { return bboxes; };
    };

    /**
     * @brief
     *
     */
    class ComputeTangentVisitor : public osg::NodeVisitor
    {
      public:
        void apply(osg::Node &node)
        {
            traverse(node);
        }

        void apply(osg::Geode &node)
        {
            for (unsigned int i = 0; i < node.getNumDrawables(); ++i)
            {
                osg::Geometry *geom = dynamic_cast<osg::Geometry *>(node.getDrawable(i));
                if (geom)
                    generateTangentArray(geom);
            }
            traverse(node);
        }

        void generateTangentArray(osg::Geometry *geom)
        {
            osg::ref_ptr<osgUtil::TangentSpaceGenerator> tsg = new osgUtil::TangentSpaceGenerator;
            tsg->generate(geom, 0);
            geom->setVertexAttribArray(6, tsg->getTangentArray());
            geom->setVertexAttribBinding(6, osg::Geometry::BIND_PER_VERTEX);
            geom->setVertexAttribArray(7, tsg->getBinormalArray());
            geom->setVertexAttribBinding(7, osg::Geometry::BIND_PER_VERTEX);
            geom->setVertexAttribArray(15, tsg->getNormalArray());
            geom->setVertexAttribBinding(15, osg::Geometry::BIND_PER_VERTEX);
        }
    };

    /**
     * @brief
     *
     */
    template <typename T>
    void setOSGImagePixel(osg::ref_ptr<osg::Image> &image,
                          unsigned int x,
                          unsigned int y,
                          unsigned int channel,
                          T value)
    {

        bool valid = (y < (unsigned int)image->s())
                     && (x < (unsigned int)image->t())
                     && (channel < (unsigned int)image->r());

        if (!valid) {
            std::cout << "Not valid" << std::endl;
            return;
        }

        uint step = (x * image->s() + y) * image->r() + channel;

        T *data = (T *)image->data();
        data = data + step;
        *data = value;
    }

    /**
     * @brief
     *
     */
    void triangles2texture(
        std::vector<Triangle> triangles,
        std::vector<uint> trianglesRef,
        std::vector<BoundingBox> bboxes,
        osg::ref_ptr<osg::Texture2D> &texture);
}

#endif
