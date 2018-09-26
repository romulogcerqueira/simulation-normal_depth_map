#include "ImageViewerCaptureTool.hpp"

// C++ includes
#include <iostream>
#include <unistd.h>

// OSG includes
#include <osg/PolygonMode>

namespace normal_depth_map {

ImageViewerCaptureTool::ImageViewerCaptureTool(osg::ref_ptr<osg::Group> node, uint width, uint height) {
    // initialize the viewer
    setupViewer(node, width, height);
}

ImageViewerCaptureTool::ImageViewerCaptureTool( osg::ref_ptr<osg::Group> node, double fovY, double fovX,
                                                uint value, bool isHeight) {
    uint width, height;

    if (isHeight) {
        height = value;
        width = height * tan(fovX * 0.5) / tan(fovY * 0.5);
    } else {
        width = value;
        height = width * tan(fovY * 0.5) / tan(fovX * 0.5);
    }

    setupViewer(node, width, height, fovY);
}

// create a RTT (render to texture) camera
osg::Camera *ImageViewerCaptureTool::createRTTCamera(osg::Camera::BufferComponent buffer, osg::Texture2D *tex, osg::GraphicsContext *gfxc)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera();
    camera->setClearColor(osg::Vec4(0, 0, 0, 1));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 0);
    camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
    camera->setGraphicsContext(gfxc);
    camera->setDrawBuffer(GL_FRONT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setViewMatrix(osg::Matrixd::identity());
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

osg::Camera* ImageViewerCaptureTool::createHUDCamera(double left, double right, double bottom, double top)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera();
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);
    camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top));
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return camera.release();
}

osg::Geode* ImageViewerCaptureTool::createScreenQuad(float width, float height, float scale)
{
    osg::Geometry *geom = osg::createTexturedQuadGeometry(
        osg::Vec3(), osg::Vec3(width, 0.0f, 0.0f),
        osg::Vec3(0.0f, height, 0.0f),
        0.0f, 0.0f, width * scale, height * scale);
    osg::ref_ptr<osg::Geode> quad = new osg::Geode;
    quad->addDrawable(geom);

    int values =
        osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
    quad->getOrCreateStateSet()->setAttribute(  new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL),
                                                values);
    quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);
    return quad.release();
}

void ImageViewerCaptureTool::setupViewer(osg::ref_ptr<osg::Group> node, uint width, uint height, double fovY)
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

    // set subgroups of main node
    osg::ref_ptr<osg::Group> pass1root = node->getChild(0)->asGroup();
    osg::ref_ptr<osg::Group> pass2root = node->getChild(1)->asGroup();
    osg::ref_ptr<osg::Group> scene = new osg::Group();
    for (size_t i = 2; i < node->getNumChildren(); i++) {
        scene->addChild(node->getChild(i));
    }

    // 1st pass: primary reflections by rasterization pipeline
    osg::ref_ptr<osg::Texture2D> pass12tex0 = createFloatTexture(width, height);
    osg::ref_ptr<osg::Camera> pass1cam = createRTTCamera(osg::Camera::COLOR_BUFFER0, pass12tex0, gfxc);
    pass1cam->addChild(scene);
    pass1root->addChild(pass1cam);

    // 2nd pass: secondary reflections by ray-triangle intersection
    osg::ref_ptr<osg::Texture2D> pass2tex = createFloatTexture(width, height);
    osg::ref_ptr<osg::Camera> pass2cam = createRTTCamera(osg::Camera::COLOR_BUFFER0, pass2tex, gfxc);
    pass2cam->addChild(scene);

    // set the first pass textures as uniform of second pass
    osg::ref_ptr<osg::StateSet> pass2state = pass2cam->getOrCreateStateSet();

    pass2state->addUniform(new osg::Uniform("firstReflectionTex", 1));
    pass2state->setTextureAttributeAndModes(1, pass12tex0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    pass2state->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC2, "rttTexSize"));
    pass2state->getUniform("rttTexSize")->set(osg::Vec2(width * 1.0, height * 1.0));
    pass2root->addChild(pass2cam);

    // setup post render camera
    osg::ref_ptr<osg::Camera> hudCamera = createHUDCamera(0.0, 1.0, 0.0, 1.0);
    hudCamera->addChild(createScreenQuad(1.0f, 1.0f));
    hudCamera->getOrCreateStateSet()->setTextureAttributeAndModes(0, pass2tex);

    // setup the root scene
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(pass1root.get());
    root->addChild(pass2root.get());
    root->addChild(hudCamera.get());

    // setup viewer
    _viewer = new osgViewer::Viewer;
    _viewer->getCamera()->setViewport(0, 0, width, height);
    _viewer->getCamera()->setGraphicsContext(gfxc);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(osg::RadiansToDegrees(fovY), (width * 1.0 / height), 0.1, 1000);
    _viewer->setSceneData(root);

    // render texture to image
    _capture = new WindowCaptureScreen(gfxc, pass2tex);
    pass2cam->setFinalDrawCallback(_capture);
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::grabImage() {
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
