#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string object_list_file_dir;
int maxObjectNum = 100;
double broadcastDisThre = 2.0;
double broadcastRate = 5.0;

float vehicleX = 0, vehicleY = 0;
double curTime = 0, sendTime = 0;

void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bboxBroadcast");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("object_list_file_dir", object_list_file_dir);
  nhPrivate.getParam("maxObjectNum", maxObjectNum);
  nhPrivate.getParam("broadcastDisThre", broadcastDisThre);
  nhPrivate.getParam("broadcastRate", broadcastRate);

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  ros::Publisher pubObjectMarker = nh.advertise<visualization_msgs::MarkerArray>("object_markers", 5);
  visualization_msgs::MarkerArray objectMarkerArray;

  const int objNumConst = maxObjectNum;
  float objMidX[objNumConst] = {0};
  float objMidY[objNumConst] = {0};
  float objMidZ[objNumConst] = {0};
  float objL[objNumConst] = {0};
  float objH[objNumConst] = {0};
  float objW[objNumConst] = {0};
  float objHeading[objNumConst] = {0};
  string objLabel[objNumConst];
  int objExist[objNumConst] = {0};
  int objValid[objNumConst] = {0};

  FILE *object_list_file = fopen(object_list_file_dir.c_str(), "r");
  if (object_list_file == NULL) {
    printf ("\nCannot read object_list_with_labels.txt file, exit.\n\n");
    exit(1);
  }

  int objID;
  char s[100], s2[100];
  int val1, val2, val3, val4, val5, val6, val7, val8, val9;
  float x, y, z, l, w, h, heading;
  while (1) {
    val1 = fscanf(object_list_file, "%d", &objID);
    val2 = fscanf(object_list_file, "%f", &x);
    val3 = fscanf(object_list_file, "%f", &y);
    val4 = fscanf(object_list_file, "%f", &z);
    val5 = fscanf(object_list_file, "%f", &l);
    val6 = fscanf(object_list_file, "%f", &w);
    val7 = fscanf(object_list_file, "%f", &h);
    val8 = fscanf(object_list_file, "%f", &heading);
    val9 = fscanf(object_list_file, "%s", s);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1 || val8 != 1 || val9 != 1) {
      break;
    }

    while (s[strlen(s) - 1] != '"') {
      val9 = fscanf(object_list_file, "%s", s2);
      
      if (val9 != 1) break;

      strcat(s, " ");
      strcat(s, s2);
    }
    
    if (objID < 0 || objID >= maxObjectNum) continue;

    objMidX[objID] = x;
    objMidY[objID] = y;
    objMidZ[objID] = z;
    objL[objID] = l;
    objW[objID] = w;
    objH[objID] = h;
    objHeading[objID] = heading;
    objExist[objID] = 1;

    for (int i = 1; s[i] != '"' && i < 100; i++) objLabel[objID] += s[i];
  }
  
  fclose(object_list_file);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (curTime - sendTime > 1.0 / broadcastRate) {
      int objValidNum = 0;
      for (int j = 0; j < maxObjectNum; j++) {
        if (objExist[j] == 1) {
          float disX = objMidX[j] - vehicleX;
          float disY = objMidY[j] - vehicleY;
          float dis = sqrt(disX * disX + disY * disY);

          if (dis < broadcastDisThre) {
            objValid[j] = 1;
            objValidNum++;
          } else {
            objValid[j] = 0;
          }
        }
      }

      if (objValidNum > 0) {
        objectMarkerArray.markers.resize(objValidNum);

        int objValidCount = 0;
        for (int j = 0; j < maxObjectNum; j++) {
          if (objValid[j] == 1) {
            objectMarkerArray.markers[objValidCount].header.frame_id = "map";
            objectMarkerArray.markers[objValidCount].header.stamp = ros::Time().fromSec(curTime);
            objectMarkerArray.markers[objValidCount].ns = objLabel[j];
            objectMarkerArray.markers[objValidCount].id = j;
            objectMarkerArray.markers[objValidCount].action = visualization_msgs::Marker::ADD;
            objectMarkerArray.markers[objValidCount].type = visualization_msgs::Marker::CUBE;
            objectMarkerArray.markers[objValidCount].pose.position.x = objMidX[j];
            objectMarkerArray.markers[objValidCount].pose.position.y = objMidY[j];
            objectMarkerArray.markers[objValidCount].pose.position.z = objMidZ[j];
            objectMarkerArray.markers[objValidCount].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, objHeading[j]);
            objectMarkerArray.markers[objValidCount].scale.x = objL[j];
            objectMarkerArray.markers[objValidCount].scale.y = objW[j];
            objectMarkerArray.markers[objValidCount].scale.z = objH[j];
            objectMarkerArray.markers[objValidCount].color.a = 0.5;
            objectMarkerArray.markers[objValidCount].color.r = 1.0;
            objectMarkerArray.markers[objValidCount].color.g = 0;
            objectMarkerArray.markers[objValidCount].color.b = 0;
            objValidCount++;
          }
        }

        pubObjectMarker.publish(objectMarkerArray);
      }

      sendTime = curTime;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
