#include "Tools.hpp"

// C++ includes
#include <cmath>
#include <iostream>

namespace normal_depth_map {

double underwaterSignalAttenuation( const double frequency,
                                    const double temperature,
                                    const double depth,
                                    const double salinity,
                                    const double acidity) {

    double frequency2 = frequency * frequency;

    // borid acid and magnesium sulphate relaxation frequencies (in kHz)
    double f1 = 0.78 * pow(salinity / 35, 0.5) * exp(temperature / 26);
    double f2 = 42 * exp(temperature / 17);

    // borid acid contribution
    double borid = 0.106 * ((f1 * frequency2) / (frequency2 + f1 * f1)) * exp((acidity - 8) / 0.56);

    // magnesium sulphate contribuion
    double magnesium = 0.52 * (1 + temperature / 43) * (salinity / 35)
                        * ((f2 * frequency2) / (frequency2 + f2 * f2)) * exp(-depth / 6000);

    // freshwater contribution
    double freshwater = 0.00049 * frequency2 * exp(-(temperature / 27 + depth / 17000));

    // absorptium attenuation coefficient in dB/km
    double attenuation = borid + magnesium + freshwater;

    // convert dB/km to dB/m
    attenuation = attenuation / 1000.0;

    // convert dB/m to Pa/m
    attenuation = pow(10, -attenuation / 20);
    attenuation = -log(attenuation);

    return attenuation;
}

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
