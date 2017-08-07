# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

rock_init(normal_depth_map 0.1)
rock_standard_layout()

install(DIRECTORY resources/shaders/
    DESTINATION share/normal_depth_map/shaders)
