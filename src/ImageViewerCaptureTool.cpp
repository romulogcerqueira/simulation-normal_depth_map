#include "ImageViewerCaptureTool.hpp"

// C++ includes
#include <iostream>
#include <unistd.h>

// OSG includes
#include <osg/PolygonMode>

namespace normal_depth_map {

ImageViewerCaptureTool::ImageViewerCaptureTool(uint width, uint height)
{
    // initialize the viewer
    setupViewer(width, height);
}

ImageViewerCaptureTool::ImageViewerCaptureTool( double fovY, double fovX,
                                                uint value, bool isHeight) {
    uint width, height;

    if (isHeight) {
        height = value;
        width = height * tan(fovX * 0.5) / tan(fovY * 0.5);
    } else {
        width = value;
        height = width * tan(fovY * 0.5) / tan(fovX * 0.5);
    }

    setupViewer(width, height, fovY);
}

// create a RTT (render to texture) camera
osg::Camera *ImageViewerCaptureTool::createRTTCamera(osg::Camera* cam, osg::Camera::BufferComponent buffer, osg::Texture2D *tex, osg::GraphicsContext *gfxc)
{
    osg::ref_ptr<osg::Camera> camera = cam;
    camera->setClearColor(osg::Vec4(0, 0, 0, 1));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 0);
    camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
    camera->setGraphicsContext(gfxc);
    camera->setDrawBuffer(GL_FRONT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->attach(buffer, tex);
    return camera.release();
}

// create float textures to be rendered in FBO
osg::Texture2D* ImageViewerCaptureTool::createFloatTexture(uint width, uint height)
{
    osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
    tex2D->setTextureSize( width, height );
    tex2D->setInternalFormat( GL_RGB32F_ARB );
    tex2D->setSourceFormat( GL_RGBA );
    tex2D->setSourceType( GL_FLOAT );
    tex2D->setResizeNonPowerOfTwoHint( false );
    tex2D->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
    tex2D->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
    return tex2D.release();
}

void ImageViewerCaptureTool::setupViewer(uint width, uint height, double fovY)
{
    // set graphics contexts
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->pbuffer = true;
    traits->readDISPLAY();
    osg::ref_ptr<osg::GraphicsContext> gfxc = osg::GraphicsContext::createGraphicsContext(traits.get());

    // set the main camera
    _viewer = new osgViewer::Viewer;
    osg::ref_ptr<osg::Texture2D> tex = createFloatTexture(width, height);
    osg::ref_ptr<osg::Camera> cam = createRTTCamera(_viewer->getCamera(), osg::Camera::COLOR_BUFFER0, tex, gfxc);
    cam->setProjectionMatrixAsPerspective(osg::RadiansToDegrees(fovY), (width * 1.0 / height), 0.1, 1000);

    // render texture to image
    _capture = new WindowCaptureScreen(gfxc, tex);
    cam->setFinalDrawCallback(_capture);
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::grabImage(osg::ref_ptr<osg::Node> node)
{
    // set the current node
    _viewer->setSceneData(node);

    // if the view matrix is invalid (NaN), use the identity
    if (_viewer->getCamera()->getViewMatrix().isNaN())
        _viewer->getCamera()->setViewMatrix(osg::Matrixd::identity());

    // grab the current frame
    _viewer->frame();
    return _capture->captureImage();
}

void ImageViewerCaptureTool::setCameraPosition( const osg::Vec3d& eye,
                                                const osg::Vec3d& center,
                                                const osg::Vec3d& up) {

    _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::getCameraPosition( osg::Vec3d& eye,
                                                osg::Vec3d& center,
                                                osg::Vec3d& up) {

    _viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::setBackgroundColor(osg::Vec4d color) {
    _viewer->getCamera()->setClearColor(color);
}

////////////////////////////////
////WindowCaptureScreen METHODS
////////////////////////////////

WindowCaptureScreen::WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gfxc, osg::Texture2D* tex) {
    _mutex = new OpenThreads::Mutex();
    _condition = new OpenThreads::Condition();
    _image = new osg::Image();

    // checks the GraficContext from the camera viewer
    if (gfxc->getTraits()) {
        _tex = tex;
        int width = gfxc->getTraits()->width;
        int height = gfxc->getTraits()->height;
        _image->allocateImage(width, height, 4, GL_RGBA, GL_FLOAT);
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
    osg::ref_ptr<osg::GraphicsContext> gfxc = renderInfo.getState()->getGraphicsContext();

    if (gfxc->getTraits()) {
        _mutex->lock();

        // read the color buffer as 32-bit floating point
        renderInfo.getState()->applyTextureAttribute(0, _tex);
        _image->readImageFromCurrentTexture(renderInfo.getContextID(), true, GL_FLOAT);

        // grants the access to image
        _condition->signal();
        _mutex->unlock();
    }
}
}
