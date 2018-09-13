find_package( OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgFX osgGA)

add_library(sonar_simulation SHARED
  ImageViewerCaptureTool.cpp
  NormalDepthMap.cpp
  Tools.cpp)

set(source_libs_local  sonar_simulation  ${OPENSCENEGRAPH_LIBRARIES})
set(source_libs ${source_libs_local} PARENT_SCOPE)

configure_file(${source_path}/Definitions.hpp.in
               ${PROJECT_BINARY_DIR}/generated/Definitions.hpp)
include_directories(${PROJECT_BINARY_DIR}/generated/)

install (TARGETS sonar_simulation
         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)

install (FILES ${PROJECT_BINARY_DIR}/generated/Definitions.hpp  
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${CMAKE_PROJECT_NAME}/)

file(GLOB header_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

foreach(header_file ${header_files})
  install (FILES ${header_file}
           DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${CMAKE_PROJECT_NAME}/)
endforeach(header_file)