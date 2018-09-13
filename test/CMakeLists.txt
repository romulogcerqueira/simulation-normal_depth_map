find_package(OpenCV 3.3 REQUIRED)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(test_tools SHARED TestHelper.cpp)
set(test_libs test_tools ${OpenCV_LIBS} ${Boost_LIBRARIES})

file(GLOB test_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *test.cpp)

foreach(test_src ${test_files})
        get_filename_component(test_name ${test_src} NAME_WE)

        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} ${test_libs}
                                           ${source_libs})

        set_target_properties(${test_name} PROPERTIES 
                 RUNTIME_OUTPUT_DIRECTORY ${bin_path}/testbin)

        add_test(NAME ${test_name} 
                WORKING_DIRECTORY ${bin_path}/testbin 
                COMMAND ${bin_path}/testbin/${test_name})
endforeach(test_src)