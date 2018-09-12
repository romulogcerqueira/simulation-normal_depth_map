configure_file(${source_path}/Definitions.hpp.in
               ${PROJECT_BINARY_DIR}/generated/Definitions.hpp)
include_directories(${PROJECT_BINARY_DIR}/generated/)

find_package( OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgFX osgGA)

add_library(sonar_simulation SHARED
  ImageViewerCaptureTool.cpp
  NormalDepthMap.cpp
  Tools.cpp)

set(source_libs_local  sonar_simulation  ${OPENSCENEGRAPH_LIBRARIES})
set(source_libs ${source_libs_local} PARENT_SCOPE)