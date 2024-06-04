set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  sensor_msgs
  )
find_package(choreonoid REQUIRED)

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

include_directories(${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
rtmbuild_add_library(JointStateROSBridge SHARED JointStateROSBridge.cpp)
target_link_libraries(JointStateROSBridge ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES_ABS})
set_target_properties(JointStateROSBridge PROPERTIES PREFIX "") # libCommandLevelROSBridge.so -> CommandLevelROSBridge.so

rtmbuild_add_executable(JointStateROSBridgeComp JointStateROSBridgeComp.cpp)
target_link_libraries(JointStateROSBridgeComp JointStateROSBridge)
