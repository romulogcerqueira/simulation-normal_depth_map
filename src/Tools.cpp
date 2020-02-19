#include "Tools.hpp"

// C++ includes
#include <cmath>
#include <iostream>

namespace normal_depth_map {

void triangles2texture(
    std::vector<Triangle> triangles,
    std::vector<uint> trianglesRef,
    std::vector<BoundingBox> bboxes,
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
    for(size_t j = 0; j < triangles.size(); j++)
    {
        std::vector<float> data = triangles[j].getAllDataAsVector();
        for (size_t i = 0; i < data.size(); i++)
            setOSGImagePixel(image, i, j, 0, data[i]);
    }

    // set triangles reference into texture
    for(size_t i = 0; i < trianglesRef.size(); i++)
    {
        size_t idx = (triangles.size() + i);
        setOSGImagePixel(image, 0, idx, 0, (float) trianglesRef[i]);
    }

    // set bounding boxes into texture
    for (size_t j = 0; j < bboxes.size(); j++)
    {
        size_t idx = (triangles.size() + trianglesRef.size() + j);
        std::vector<float> data = bboxes[j].getAllDataAsVector();

        for (size_t i = 0; i < data.size(); i++)
            setOSGImagePixel(image, i, idx, 0, data[i]);

    }

    // set texture
    texture = new osg::Texture2D;
    texture->setTextureSize(image->s(), image->t());
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setUnRefImageDataAfterApply(true);
    texture->setImage(image);
}
}
