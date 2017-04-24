cmake_minimum_required(VERSION 3.5)
project(SimplerThanWaldo)

find_package(OpenCV REQUIRED)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++14")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lstdc++fs")
endif()

include_directories(Headers/
                    ${OpenCV_INCLUDE_DIRS})

file(GLOB PROJECT_HEADERS Headers/*.hpp
                          Headers/*.h)
file(GLOB PROJECT_SOURCES Sources/*.cpp)
file(GLOB PROJECT_CONFIGS CMakeLists.txt
                          README.md
                         .gitignore)

source_group("Sources" FILES ${PROJECT_SOURCES})
source_group("Headers" FILES ${PROJECT_HEADERS})

add_definitions(-DDEBUG
                -DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

add_executable( ${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS} )


add_library(fs STATIC IMPORTED)
set_target_properties(fs PROPERTIES IMPORTED_LOCATION /usr/lib/libboost_unit_test_framework.a)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

#needs 3.5+
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME})
