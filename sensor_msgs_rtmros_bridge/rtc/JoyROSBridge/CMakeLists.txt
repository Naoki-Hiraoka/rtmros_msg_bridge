add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  sensor_msgs
  )

include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(JoyROSBridge SHARED JoyROSBridge.cpp)
target_link_libraries(JoyROSBridge ${catkin_LIBRARIES})
set_target_properties(JoyROSBridge PROPERTIES PREFIX "") # libJoyROSBridge.so -> JoyROSBridge.so

rtmbuild_add_executable(JoyROSBridgeComp JoyROSBridgeComp.cpp)
target_link_libraries(JoyROSBridgeComp JoyROSBridge)
