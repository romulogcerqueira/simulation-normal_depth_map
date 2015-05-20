/*
 * ImageCaptureTool.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: tiagotrocoli
 */

#include "ImageViewerCaptureTool.hpp"
#include <iostream>
#include <unistd.h>
#include <osgDB/WriteFile>

namespace vizkit3d_normal_depth_map {

ImageViewerCaptureTool::ImageViewerCaptureTool(uint width, uint height) {

    // initialize the hide viewer;
    initializeProperties(width, height);
}

ImageViewerCaptureTool::ImageViewerCaptureTool(double fovY, double fovX, uint height) {
    double aspectRatio = fovX / fovY;
    uint width = height * aspectRatio;
    initializeProperties(width, height);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(fovY, aspectRatio, 0.01, 1000);
}

void ImageViewerCaptureTool::initializeProperties(uint width, uint height) {
    // initialize the hide viewer;
    _viewer = new osgViewer::Viewer;
    osg::Camera *camera = this->_viewer->getCamera();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->width = width;
    traits->height = height;
    traits->pbuffer = true;
    traits->readDISPLAY();
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    camera->setGraphicsContext(gc);
    camera->setDrawBuffer(GL_FRONT);
    camera->setViewport(new osg::Viewport(0, 0, width, height));
    // initialize the class to get the image in float data resolution
    _capture = new WindowCaptureScreen(gc);
    _viewer->getCamera()->setFinalDrawCallback(_capture);
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::grabImage(osg::ref_ptr<osg::Node> node) {
    _viewer->setSceneData(node);
    _viewer->frame();
    return _capture->captureImage();
}

void ImageViewerCaptureTool::setCameraPosition(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up) {
    _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::getCameraPosition(osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up) {
    _viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::setBackgroundColor(osg::Vec4d color) {
    _viewer->getCamera()->setClearColor(color);
}

////////////////////////////////
////WindowCaptureScreen METHODS
////////////////////////////////

WindowCaptureScreen::WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gc) {

    _mutex = new OpenThreads::Mutex();
    _condition = new OpenThreads::Condition();
    _image = new osg::Image();

    // checks the GraficContext from the camera viewer
    if (gc->getTraits()) {

        GLenum pixelFormat;
        if (gc->getTraits()->alpha)
            pixelFormat = GL_RGBA;
        else
            pixelFormat = GL_RGB;

        int width = gc->getTraits()->width;
        int height = gc->getTraits()->height;

        // allocates the image memory space
        _image->allocateImage(width, height, 1, pixelFormat, GL_FLOAT);
    }
}

WindowCaptureScreen::~WindowCaptureScreen() {
    delete (_condition);
    delete (_mutex);
}

osg::ref_ptr<osg::Image> WindowCaptureScreen::captureImage() {

    //wait to finish the capture image in call back
    _condition->wait(_mutex);
    return _image;
}

void WindowCaptureScreen::operator ()(osg::RenderInfo& renderInfo) const {
    osg::ref_ptr<osg::GraphicsContext> gc = renderInfo.getState()->getGraphicsContext();
    if (gc->getTraits()) {
        _mutex->lock();
        _image->readPixels(0, 0, _image->s(), _image->t(), _image->getPixelFormat(), GL_FLOAT);
        //grants the access to image
        _condition->signal();
        _mutex->unlock();
    }
}

}
