cmake_minimum_required(VERSION 3.8)
project(stereofisheye_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED
            tf2 
            tf2_ros
            std_msgs
            sensor_msgs
            cv_bridge
            geometry_msgs
            image_transport
            OpenCV REQUIRED)

set(  SVS_PATH
/home/roser/repos/fisheye_stereo/fy_lib_source
)           

include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${SVS_PATH}/include
  )

  # ${SVS_PATH}/src
# )
set(DEWARPER_SRCS
	${SVS_PATH}/src/FisheyeDewarper.cpp
	${SVS_PATH}/src/SurroundSystem.cpp
	${SVS_PATH}/src/StereoPair.cpp
	${SVS_PATH}/src/Utils.cpp
	${SVS_PATH}/src/models/CameraModel.cpp
	${SVS_PATH}/src/models/Scaramuzza.cpp
	${SVS_PATH}/src/models/Pinhole.cpp
	${SVS_PATH}/src/models/Atan.cpp
	${SVS_PATH}/src/models/RealAtan.cpp
	${SVS_PATH}/src/models/Mei.cpp
	${SVS_PATH}/src/models/KannalaB.cpp
	${SVS_PATH}/src/models/DoubleSphere.cpp
  	${SVS_PATH}/include/CamDesc.hpp
	${SVS_PATH}/include/CameraModel.hpp
	${SVS_PATH}/include/FisheyeDewarper.hpp
	${SVS_PATH}/include/models.hpp
  	${SVS_PATH}/include/Intrinsics.hpp
	${SVS_PATH}/include/SurroundSystem.hpp
	${SVS_PATH}/include/StereoPair.hpp
	${SVS_PATH}/include/Utils.hpp
)

add_library(svs SHARED ${DEWARPER_SRCS})
target_link_libraries( svs ${OpenCV_LIBS})  

add_executable(${PROJECT_NAME}_node src/sfyNode.cpp)
# add_executable(odo_repub_node src/odometry_repub.cpp)

ament_target_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} rclcpp)
# ament_target_dependencies(odo_repub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  svs
)
# target_link_libraries(odo_repub_node
#   ${catkin_LIBRARIES}
# )



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
