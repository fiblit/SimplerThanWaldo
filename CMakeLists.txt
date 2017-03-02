project(SimplerThanWaldo)

find_package( OpenCV REQUIRED )

include_directories(${PROJECT_NAME}/Headers/
                    ${OpenCV_INCLUDE_DIRS})

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++14")
endif()

file(GLOB PROJECT_HEADERS ${PROJECT_NAME}/Headers/*.hpp)
file(GLOB PROJECT_SOURCES ${PROJECT_NAME}/Sources/*.cpp)
file(GLOB PROJECT_CONFIGS CMakeLists.txt
                          README.md
                         .gitignore)

source_group("Sources" FILES ${PROJECT_SOURCES})
source_group("Headers" FILES ${PROJECT_HEADERS})

add_definitions(-DDEBUG)

add_executable( ${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME})