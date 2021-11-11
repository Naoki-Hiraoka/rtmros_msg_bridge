add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  nav_msgs
  tf2
  )

include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(WrenchStampedROSBridge SHARED WrenchStampedROSBridge.cpp)
target_link_libraries(WrenchStampedROSBridge ${catkin_LIBRARIES})
set_target_properties(WrenchStampedROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

rtmbuild_add_executable(WrenchStampedROSBridgeComp WrenchStampedROSBridgeComp.cpp)
target_link_libraries(WrenchStampedROSBridgeComp WrenchStampedROSBridge)
