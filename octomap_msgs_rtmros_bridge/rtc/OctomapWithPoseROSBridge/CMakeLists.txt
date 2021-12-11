add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  octomap_msgs
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/.. # for /idl
  )
rtmbuild_add_executable(OctomapWithPoseROSBridgeComp OctomapWithPoseROSBridgeComp.cpp OctomapWithPoseROSBridge.cpp)
target_link_libraries(OctomapWithPoseROSBridgeComp ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
add_dependencies(OctomapWithPoseROSBridgeComp RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl
