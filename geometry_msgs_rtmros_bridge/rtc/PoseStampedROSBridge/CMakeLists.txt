add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  geometry_msgs
  tf2
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
rtmbuild_add_library(PoseStampedROSBridge SHARED PoseStampedROSBridge.cpp)
target_link_libraries(PoseStampedROSBridge ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
set_target_properties(PoseStampedROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

rtmbuild_add_executable(PoseStampedROSBridgeComp PoseStampedROSBridgeComp.cpp)
target_link_libraries(PoseStampedROSBridgeComp PoseStampedROSBridge)
