cmake_minimum_required(VERSION 3.8)
project(ros2_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wl,-Bstatic -lrpc -lboost_filesystem -Wl,-Bdynamic -lpng -ltiff -ljpeg -lRecast -lDetour -lDetourCrowd)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(TIFF REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Boost
)


include(dependency.cmake)

add_executable(${PROJECT_NAME}
    src/main.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
)


ament_target_dependencies(${PROJECT_NAME} ${ament_dependencies} )

target_include_directories(${PROJECT_NAME} PUBLIC
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_directories(${PROJECT_NAME} PUBLIC
    ${libCarla_LIB_DIR}
)

target_link_libraries(${PROJECT_NAME} ${libCarla_LIBS} ${Boost_LIBRARIES} ${TIFF_LIBRARIES} jpeg png
)



install(TARGETS
${PROJECT_NAME} 
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY libcarla-install/lib/
  DESTINATION lib
)

ament_package()