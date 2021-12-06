/*!*******************************************************************************************
 *  \file       inventory_item_belief_updater.cpp
 *  \brief      distance measurement implementation file.
 *  \details    This file contains the DistanceMeasurement implementattion of the class.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#include "../include/inventory_item_image_annotator.h"
#include <pluginlib/class_list_macros.h>


InventoryItemImageAnnotator::InventoryItemImageAnnotator(){
}

InventoryItemImageAnnotator::~InventoryItemImageAnnotator(){
}

void InventoryItemImageAnnotator::ownSetUp(){
    ros::NodeHandle nh("~");
    nh.param<std::string>("camera_topic", camera_topic_str, "usb_cam/image_raw");
    nh.param<std::string>("item_annotator_topic", item_annotator_topic_str, "item_annotator");
    nh.param<std::string>("qr_camera_topic", camera_bounding_topic_str, "bounding_image_raw");
    item_annotator_sub = node_handle.subscribe(item_annotator_topic_str, 30, &InventoryItemImageAnnotator::AnnotatorCallback, this);
    camera_sub = node_handle.subscribe(camera_topic_str, 1, &InventoryItemImageAnnotator::CameraCallback, this);
    qr_camera_pub = node_handle.advertise<sensor_msgs::Image>(camera_bounding_topic_str, 30, true);

}

void InventoryItemImageAnnotator::CameraCallback (const sensor_msgs::ImageConstPtr &msg){
  mtx.lock();
  try
  {
    image_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  mtx.unlock();
  // ToDo := If mutex implementation doesn't unlock upon destruction or destruction doesn't happen
  // when going through the catch clause, there will be a deadlock
}



void InventoryItemImageAnnotator::AnnotatorCallback (const aerostack_msgs::ListOfInventoryItemAnnotation &msg){
    aerostack_msgs::ListOfInventoryItemAnnotation annotationList;
    annotationList = msg;
    for (int i = 0; i < annotationList.list_of_annotations.size(); i++){
        std::vector<cv::Point2f> points (4);
        points[0] = cv::Point2f(annotationList.list_of_annotations[i].bounding_points[0].x, annotationList.list_of_annotations[i].bounding_points[0].y);
        points[1] = cv::Point2f(annotationList.list_of_annotations[i].bounding_points[1].x, annotationList.list_of_annotations[i].bounding_points[1].y);
        points[2] = cv::Point2f(annotationList.list_of_annotations[i].bounding_points[2].x, annotationList.list_of_annotations[i].bounding_points[2].y);
        points[3] = cv::Point2f(annotationList.list_of_annotations[i].bounding_points[3].x, annotationList.list_of_annotations[i].bounding_points[3].y);
        cv::RotatedRect box = cv::minAreaRect(points);
        BoundingBox = box.boundingRect();
        cv::rectangle(image_cv->image, BoundingBox, cv::Scalar(255,255,0), 5);
        cv::putText(image_cv->image, annotationList.list_of_annotations[i].object, cv::Point(BoundingBox.tl().x, BoundingBox.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,0), 2); 
    }
    qr_camera_pub.publish(image_cv->toImageMsg());
}

void InventoryItemImageAnnotator::ownStart(){
}

void InventoryItemImageAnnotator::ownRun(){
}

void InventoryItemImageAnnotator::ownStop(){

}


