rock_library(vizkit3d_normal_depth_map
    SOURCES NormalDepthMap.cpp ImageViewerCaptureTool.cpp FXAA.cpp
    HEADERS NormalDepthMap.hpp ImageViewerCaptureTool.hpp FXAA.hpp
    DEPS_PKGCONFIG openscenegraph)
