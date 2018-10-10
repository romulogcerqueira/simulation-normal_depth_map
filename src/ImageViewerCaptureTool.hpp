#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_

// OSG includes
#include <osgViewer/Viewer>
#include <osg/Texture2D>

namespace normal_depth_map {

/**
 * Capture the osg image from a node scene without GUI presentation.
 */
class WindowCaptureScreen: public osg::Camera::DrawCallback {
public:

    /**
     * This class allow access raw data image from the viewer with callback function.
     *
     * @param gc: viewer's graphics context.
     * @param tex: RTT buffer.
     */
	WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gfxc, osg::Texture2D *tex);
	~WindowCaptureScreen();

    /**
     * Read the osg image RTT buffer, and synchronizes the osg threads before return the image.
     *
     * @return the osg image from the scene with defined camera and view parameters.
     */
	osg::ref_ptr<osg::Image> captureImage();

private:

    /**
     * Overload function to read the buffer and store on osg image.
     */
    void operator ()(osg::RenderInfo& renderInfo) const;

    // threads to control the read/write access of rendered osg image
    OpenThreads::Mutex *_mutex;
    OpenThreads::Condition *_condition;

    // rendered osg image
    osg::ref_ptr<osg::Image> _image;

    // RTT buffer
    osg::ref_ptr<osg::Texture2D> _tex;
};

class ImageViewerCaptureTool {
public:
    /**
     * This constructor generates a viewer to render the osg image without GUI.
     *
     * @param width: image columns (in pixels).
     * @param height: image rows (in pixels).
     */
    ImageViewerCaptureTool( uint width = 640,
                            uint height = 480);

    /**
     * This constructor class generate an osg image according FOV-Y, FOV-X and height/width resolution.
     *
     * @param fovY: vertical field of view (in radians).
     * @param fovX: horizontal field of view (in radians).
     * @param value: height/width to generate the image.
     * @param isHeight: indicates if value is height or width.
     */
    ImageViewerCaptureTool( double fovY,
                            double fovX,
                            uint value,
                            bool isHeight = true);

    /**
     * Capture the scene (normal depth map) from camera's viewpoint and render it to osg image.
     *
     * @param node: the normal depth map node.
     * @return the rendered image from main node.
     */
    osg::ref_ptr<osg::Image> grabImage(osg::ref_ptr<osg::Node> node);

    void setCameraPosition(const osg::Vec3d &eye, const osg::Vec3d &center, const osg::Vec3d &up) {
        _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
    }

    void getCameraPosition(osg::Vec3d &eye, osg::Vec3d &center, osg::Vec3d &up) {
        _viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    }

    void setBackgroundColor(osg::Vec4d color) {
        _viewer->getCamera()->setClearColor(color);
    }

protected:
    /**
     * Set the viewer with desired parameters.
     *
     * @param width: image columns (in pixels).
     * @param height: image rows (in pixels).
     * @param fovY: vertical field-of-view.
     */
    void setupViewer(uint width, uint height, double fovY = (M_PI / 3));

    /**
     * Create float texture to be rendered in FBO.
     *
     * @param width: texture columns (in pixels).
     * @param height: texture rows (in pixels).
     * @return the FBO texture.
     */
    osg::Texture2D* createFloatTexture( uint width, uint height );

    /**
     * Setup a camera with an attached RTT texture.
     *
     * @param cam: the target camera.
     * @param buffer: desired buffer component.
     * @param tex: the FBO texture.
     * @param gfxc: the graphics context.
     * @return the osg camera.
     */
    osg::Camera* createRTTCamera(   osg::Camera* cam,
                                    osg::Camera::BufferComponent buffer,
                                    osg::Texture2D* tex,
                                    osg::GraphicsContext *gfxc );

    osg::ref_ptr<WindowCaptureScreen> _capture;
    osg::ref_ptr<osgViewer::Viewer> _viewer;
};

} /* namespace normal_depth_map */

#endif
