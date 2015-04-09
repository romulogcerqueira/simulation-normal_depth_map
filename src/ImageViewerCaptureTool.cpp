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
    this->_viewer = new osgViewer::Viewer;
    osg::Camera *camera = this->_viewer->getCamera();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->width = width;
    traits->height = height;
    traits->pbuffer = true;
    traits->readDISPLAY();
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    camera->setGraphicsContext(gc);
    camera->setDrawBuffer(GL_FRONT);
    // set the image resolution
    camera->setViewport(new osg::Viewport(0, 0, width, height));
    //    camera->setProjectionMatrixAsPerspective(22, 1, 0.1, 1000);

    // initializes the class to get the image in float resolution
    this->_capture = new WindowCaptureScreen(gc);
    this->_viewer->getCamera()->setFinalDrawCallback(this->_capture);
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::grabImage(osg::ref_ptr<osg::Node> node) {
    this->_viewer->setSceneData(node);
    this->_viewer->frame();
    return this->_capture->captureImage();
}

void ImageViewerCaptureTool::setCameraPosition(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up) {
    this->_viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::getCameraPosition(osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up) {
    this->_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::setBackgroundColor(osg::Vec4d color) {
    this->_viewer->getCamera()->setClearColor(color);
}

////////////////////////////////
////WindowCaptureScreen METHODS
////////////////////////////////

WindowCaptureScreen::WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gc) {

    this->_mutex = new OpenThreads::Mutex();
    this->_condition = new OpenThreads::Condition();
    this->_image = new osg::Image();

    // checks the graficcontext from the camera viewer
    if (gc->getTraits()) {

        GLenum pixelFormat;
        if (gc->getTraits()->alpha)
            pixelFormat = GL_RGBA;
        else
            pixelFormat = GL_RGB;

        int width = gc->getTraits()->width;
        int height = gc->getTraits()->height;

        // allocates the espace memory to image
        this->_image->allocateImage(width, height, 1, pixelFormat, GL_FLOAT);
    }
}

WindowCaptureScreen::~WindowCaptureScreen() {
    delete (this->_condition);
    delete (this->_mutex);
}

osg::ref_ptr<osg::Image> WindowCaptureScreen::captureImage() {

    //wait to finish the capture image in call back
    this->_condition->wait(_mutex);
    return this->_image;
}

void WindowCaptureScreen::operator ()(osg::RenderInfo& renderInfo) const {
    osg::ref_ptr<osg::GraphicsContext> gc = renderInfo.getState()->getGraphicsContext();
    if (gc->getTraits()) {
        this->_mutex->lock();
        this->_image->readPixels(0, 0, _image->s(), _image->t(), _image->getPixelFormat(), GL_FLOAT);
        //grants the access to image
        this->_condition->signal();
        this->_mutex->unlock();
    }
}

}

