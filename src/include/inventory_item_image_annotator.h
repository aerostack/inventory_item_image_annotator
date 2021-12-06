#ifndef INVENTORY_ITEM_BELIEF_UPDATER_H
#define INVENTORY_ITEM_BELIEF_UPDATER_H

#include <string>
#include <queue>
#include <array>
#include <list>
#include <cmath>
#include <algorithm>
#include <iterator>

// ROS
#include <ros/ros.h>

// Move base
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <aerostack_msgs/QrCodeLocalized.h>
#include <aerostack_msgs/ListOfQrCodeLocalized.h>
#include <aerostack_msgs/InventoryItemAnnotation.h>
#include <aerostack_msgs/ListOfInventoryItemAnnotation.h>

#include "std_srvs/Empty.h"
#include <belief_manager_msgs/AddBelief.h>
#include <belief_manager_msgs/QueryBelief.h>
#include <belief_manager_msgs/GenerateID.h>
#include <belief_manager_msgs/RemoveBelief.h>
#include <droneMsgsROS/QRInterpretation.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// Aerostack
#include <robot_process.h>
#include <stdio.h>
#include <math.h>
#include "zbar.h"
#include "std_msgs/Bool.h"

using namespace zbar;

class InventoryItemImageAnnotator : public RobotProcess
{

  public:
    InventoryItemImageAnnotator();
    ~InventoryItemImageAnnotator();
      void ownSetUp();
      void ownRun();
      void ownStop();
      void ownStart();

  private:

	
	ros::NodeHandle node_handle;
	//bool sendQRInterpretation(std::list<std::string>message, geometry_msgs::Point point , bool visible);

  	bool addBelief(std::string message, bool multivalued);
  	bool removeBelief(std::string message);
  	bool setupBeliefs(std::string message, geometry_msgs::Point point);
	float round(float var);
	std::string FindQrBoxName(int code);
  	int requestBeliefId();

  	ros::ServiceClient add_client;
  	ros::ServiceClient remove_client;
	ros::ServiceClient id_gen_client;
	ros::ServiceClient query_client;
  //Congfig variables
	std::string ing_box;
	bool bounding_box;
	std::list <std::list <std::string>> qr_list;
	std::list <std::list <geometry_msgs::Point>> points_list;
  	std::string qr_code_belief_id;
	std::string item_annotator_topic_str;
	std::string camera_topic_str;
	std::string camera_bounding_topic_str;

	ros::Subscriber item_annotator_sub;
	ros::Subscriber camera_sub;
	ros::Publisher qr_camera_pub;

	cv::Rect BoundingBox;
	std::string qrBox;
	cv_bridge::CvImagePtr image_cv;
    	std::mutex mtx;

  	void AnnotatorCallback (const aerostack_msgs::ListOfInventoryItemAnnotation &msg);
	void CameraCallback (const sensor_msgs::ImageConstPtr& msg);

};
#endif
