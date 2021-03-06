## Use the variable PROJECT_NAME for changing the target name
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set( PROJECT_NAME ${ProjectId} )

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(${PROJECT_NAME})

# include_directories(include)

find_package(OpenGL REQUIRED)
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLEW)
include_directories(/usr/local/opt/glew/include)
include_directories(/usr/local/opt/glfw/include)
link_directories(/usr/local/lib)

## add imgui as a library
add_library(imgui STATIC
  src/imgui/imgui.cpp
  src/imgui/imgui_demo.cpp
  src/imgui/imgui_draw.cpp
  src/imgui/imgui_widgets.cpp
  src/imgui/imgui_impl_glfw.cpp
  src/imgui/imgui_impl_opengl3.cpp
)

## Define libraries
add_library(${PROJECT_NAME}_lib
  src/struct.cpp
  src/GridTable.cpp
  src/ScanPointResampler.cpp
  src/DataAssociatorGT.cpp
  src/DataAssociator.cpp
  src/PoseEstimator.cpp
  src/RefScanMaker.cpp
  src/PointCloudMap.cpp
  src/PointCloudMapGT.cpp
  src/ScanMatcher.cpp
  src/SensorDataReader.cpp
  src/ScanPointAnalyser.cpp
  src/SlamLauncher.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
 include
)

## Define the executable
set(SRCS 
  src/main.cpp
)
add_executable(${PROJECT_NAME} ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME} 
  imgui
  glew
  glfw 
  ${OPENGL_gl_LIBRARY} 
  ${PROJECT_NAME}_lib
)


## Module testing
add_executable(resampling_test
  test/resampling_test.cpp
)
target_include_directories(resampling_test PUBLIC
  include
  src
)
target_link_libraries(resampling_test
  imgui
  glew
  glfw 
  ${OPENGL_gl_LIBRARY} 
  ${PROJECT_NAME}_lib
)

add_executable(data_associator_test
  test/data_associator_test.cpp
)
target_include_directories(data_associator_test PUBLIC
  include
  src
)
target_link_libraries(data_associator_test
  imgui
  glew
  glfw 
  ${OPENGL_gl_LIBRARY} 
  ${PROJECT_NAME}_lib
)

add_executable(scan_point_analyser_test
  test/scan_point_analyser.cpp
)
target_include_directories(scan_point_analyser_test PUBLIC
  include
  src
)
target_link_libraries(scan_point_analyser_test
  imgui
  glew
  glfw 
  ${OPENGL_gl_LIBRARY} 
  ${PROJECT_NAME}_lib
)

add_executable(grid_pcm_test
  test/grid_pcm_test.cpp
)
target_include_directories(grid_pcm_test PUBLIC
  include
  src
)
target_link_libraries(grid_pcm_test
  imgui
  glew
  glfw 
  ${OPENGL_gl_LIBRARY} 
  ${PROJECT_NAME}_lib
)

add_executable(pose_optimizer_test
  test/pose_optimizer_test.cpp
)
target_include_directories(pose_optimizer_test PUBLIC
  include
  src
)
target_link_libraries(grid_pcm_test
  ${PROJECT_NAME}_lib
)