cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME inventory_item_image_annotator)
project(${PROJECT_NAME})

add_definitions(-std=c++11)

## ================= Set project variables ==================

set(INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_DIR src/source)

set(INVENTORY_ITEM_IMAGE_ANNOTATOR_INCLUDE_DIR src/include)

set(INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_FILES
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_DIR}/inventory_item_image_annotator.cpp
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_DIR}/inventory_item_image_annotator.cpp
)

set(INVENTORY_ITEM_IMAGE_ANNOTATOR_HEADER_FILES
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_INCLUDE_DIR}/inventory_item_image_annotator.h
)

## ================= Find catkin macros and libraries =================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  droneMsgsROS
  drone_process
  robot_process
  cv_bridge
  sensor_msgs
  aerostack_msgs
  belief_manager_msgs
)
## ================= Catkin specific configuration =================

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  drone_process
  robot_process
  droneMsgsROS
  aerostack_msgs
  belief_manager_msgs
  DEPENDS yaml-cpp
)

## ================= Build =================

include_directories(${INVENTORY_ITEM_IMAGE_ANNOTATOR_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(

  inventory_item_image_annotator_lib
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_FILES}
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_HEADER_FILES}
)
add_dependencies(inventory_item_image_annotator_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(inventory_item_image_annotator_lib ${catkin_LIBRARIES})
add_executable(
  inventory_item_image_annotator
  ${INVENTORY_ITEM_IMAGE_ANNOTATOR_SOURCE_DIR}/inventory_item_image_annotator_main.cpp
)
add_dependencies(inventory_item_image_annotator ${catkin_EXPORTED_TARGETS})
target_link_libraries(inventory_item_image_annotator inventory_item_image_annotator_lib)
target_link_libraries(inventory_item_image_annotator ${catkin_LIBRARIES})
