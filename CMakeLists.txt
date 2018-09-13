cmake_minimum_required(VERSION 3.0)
project(normal_depth_map)

set(ENABLE_TEST FALSE)

## DEFINE MAIN PATHS
set(bin_path ${PROJECT_BINARY_DIR}/bin)
set(source_path ${PROJECT_SOURCE_DIR}/src)
set(resources_path ${PROJECT_SOURCE_DIR}/resources)
set(test_path ${PROJECT_SOURCE_DIR}/test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${bin_path})

include_directories(${source_path})
add_subdirectory(${source_path})

# Add test cases
if(${ENABLE_TEST})
  enable_testing()
  add_subdirectory(${test_path})
endif(${ENABLE_TEST})