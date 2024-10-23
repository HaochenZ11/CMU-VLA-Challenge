#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

string waypoint_file_dir;
string object_list_file_dir;
double waypointReachDis = 1.0;

vector<float> waypointX, waypointY, waypointHeading;

int objID;
float objMidX, objMidY, objMidZ, objL, objW, objH, objHeading;
string objLabel;

float vehicleX = 0, vehicleY = 0;

string question;

// reading waypoints from file function
void readWaypointFile()
{
  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(waypoint_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(waypoint_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  float x, y, heading;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(waypoint_file, "%f", &x);
    val2 = fscanf(waypoint_file, "%f", &y);
    val3 = fscanf(waypoint_file, "%f", &heading);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    waypointX.push_back(x);
    waypointY.push_back(y);
    waypointHeading.push_back(heading);
  }

  fclose(waypoint_file);
}

// reading objects from file function
void readObjectListFile()
{
  FILE* object_list_file = fopen(object_list_file_dir.c_str(), "r");
  if (object_list_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }
  
  char s[100], s2[100];
  int val1, val2, val3, val4, val5, val6, val7, val8, val9;
  val1 = fscanf(object_list_file, "%d", &objID);
  val2 = fscanf(object_list_file, "%f", &objMidX);
  val3 = fscanf(object_list_file, "%f", &objMidY);
  val4 = fscanf(object_list_file, "%f", &objMidZ);
  val5 = fscanf(object_list_file, "%f", &objL);
  val6 = fscanf(object_list_file, "%f", &objW);
  val7 = fscanf(object_list_file, "%f", &objH);
  val8 = fscanf(object_list_file, "%f", &objHeading);
  val9 = fscanf(object_list_file, "%s", s);
  
  if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1 || val8 != 1 || val9 != 1) {
    exit(1);
  }

  while (s[strlen(s) - 1] != '"') {
    val9 = fscanf(object_list_file, "%s", s2);
      
    if (val9 != 1) break;

    strcat(s, " ");
    strcat(s, s2);
  }

  for (int i = 1; s[i] != '"' && i < 100; i++) objLabel += s[i];
}

void pubPathWaypoints(ros::Publisher& waypointPub, geometry_msgs::Pose2D& waypointMsgs, ros::Rate& rate)
{
  int waypointID = 0;
  int waypointNum = waypointX.size();

  if (waypointNum == 0) {
    printf ("\nNo waypoint available, exit.\n\n");
    exit(1);
  }

  // publish fist waypoint
  waypointMsgs.x = waypointX[waypointID];
  waypointMsgs.y = waypointY[waypointID];
  waypointMsgs.theta = waypointHeading[waypointID];
  waypointPub.publish(waypointMsgs);

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float disX = vehicleX - waypointX[waypointID];
    float disY = vehicleY - waypointY[waypointID];

    // move to the next waypoint and publish
    if (sqrt(disX * disX + disY * disY) < waypointReachDis) {
      if (waypointID == waypointNum - 1) break;
      waypointID++;

      waypointMsgs.x = waypointX[waypointID];
      waypointMsgs.y = waypointY[waypointID];
      waypointMsgs.theta = waypointHeading[waypointID];
      waypointPub.publish(waypointMsgs);
    }

    status = ros::ok();
    rate.sleep();
  }
}

void pubObjectWaypoint(ros::Publisher& waypointPub, geometry_msgs::Pose2D& waypointMsgs)
{
  waypointMsgs.x = objMidX;
  waypointMsgs.y = objMidY;
  waypointMsgs.theta = 0;
  waypointPub.publish(waypointMsgs);
}

void pubObjectMarker(ros::Publisher& objectMarkerPub, visualization_msgs::Marker& objectMarkerMsgs)
{
  objectMarkerMsgs.header.frame_id = "map";
  objectMarkerMsgs.header.stamp = ros::Time().now();
  objectMarkerMsgs.ns = objLabel;
  objectMarkerMsgs.id = objID;
  objectMarkerMsgs.action = visualization_msgs::Marker::ADD;
  objectMarkerMsgs.type = visualization_msgs::Marker::CUBE;
  objectMarkerMsgs.pose.position.x = objMidX;
  objectMarkerMsgs.pose.position.y = objMidY;
  objectMarkerMsgs.pose.position.z = objMidZ;
  objectMarkerMsgs.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, objHeading);
  objectMarkerMsgs.scale.x = objL;
  objectMarkerMsgs.scale.y = objW;
  objectMarkerMsgs.scale.z = objH;
  objectMarkerMsgs.color.a = 0.5;
  objectMarkerMsgs.color.r = 0;
  objectMarkerMsgs.color.g = 0;
  objectMarkerMsgs.color.b = 1.0;
  objectMarkerPub.publish(objectMarkerMsgs);

}

void delObjectMarker(ros::Publisher& objectMarkerPub, visualization_msgs::Marker& objectMarkerMsgs)
{
  objectMarkerMsgs.header.frame_id = "map";
  objectMarkerMsgs.header.stamp = ros::Time().now();
  objectMarkerMsgs.ns = objLabel;
  objectMarkerMsgs.id = objID;
  objectMarkerMsgs.action = visualization_msgs::Marker::DELETE;
  objectMarkerMsgs.type = visualization_msgs::Marker::CUBE;
  objectMarkerPub.publish(objectMarkerMsgs);
}

void pubNumericalAnswer(ros::Publisher& numericalAnswerPub, std_msgs::Int32& numericalResponseMsg, int32_t numericalResponse)
{
  numericalResponseMsg.data = numericalResponse;
  numericalAnswerPub.publish(numericalResponseMsg);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
}

void questionHandler(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received question");
  question = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummyVLM");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);
  nhPrivate.getParam("object_list_file_dir", object_list_file_dir);
  nhPrivate.getParam("waypointReachDis", waypointReachDis);

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  ros::Subscriber subQuestion = nh.subscribe<std_msgs::String>("/challenge_question", 5, questionHandler);

  ros::Publisher waypointPub = nh.advertise<geometry_msgs::Pose2D> ("/way_point_with_heading", 5);
  geometry_msgs::Pose2D waypointMsgs;

  ros::Publisher objectMarkerPub = nh.advertise<visualization_msgs::Marker>("selected_object_marker", 5);
  visualization_msgs::Marker objectMarkerMsgs;

  ros::Publisher numericalAnswerPub = nh.advertise<std_msgs::Int32>("/numerical_response", 5);
  std_msgs::Int32 numericalResponseMsg;

  // read waypoints from file
  readWaypointFile();

  // read objects from file
  readObjectListFile();

  ros::Rate rate(100);

  ROS_INFO("Awaiting question...");

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    if (question.empty()) continue;
    if (question.rfind("Find", 0) == 0 || question.rfind("find", 0) == 0) {
      ROS_INFO("Marking and navigating to object.");
      pubObjectMarker(objectMarkerPub, objectMarkerMsgs);
      pubObjectWaypoint(waypointPub, waypointMsgs);
    }
    else if (question.rfind("How many", 0) == 0 || question.rfind("how many", 0) == 0) {
      delObjectMarker(objectMarkerPub, objectMarkerMsgs);
      int32_t number = (rand() % 10) + 1;
      ROS_INFO (to_string(number).c_str());
      pubNumericalAnswer(numericalAnswerPub, numericalResponseMsg, number);
    }
    else {
      delObjectMarker(objectMarkerPub, objectMarkerMsgs);
      ROS_INFO ("Navigation starts.");
      pubPathWaypoints(waypointPub, waypointMsgs, rate);
      ROS_INFO ("Navigation ends.");
    }
    question.clear();
    ROS_INFO("Awaiting question...");
  }

  return 0;
}
