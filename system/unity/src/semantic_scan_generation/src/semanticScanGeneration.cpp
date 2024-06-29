#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;
using namespace cv;

const double PI = 3.1415926;

double cameraOffsetZ = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudSeg(new pcl::PointCloud<pcl::PointXYZRGB>());

const int stackNum = 400;
float lidarXStack[stackNum];
float lidarYStack[stackNum];
float lidarZStack[stackNum];
float lidarRollStack[stackNum];
float lidarPitchStack[stackNum];
float lidarYawStack[stackNum];
double odomTimeStack[stackNum];
int odomIDPointer = -1;
int imageIDPointer = 0;

bool imageInit = false;
double imageTime = 0;

bool newLaserCloud = false;
double laserCloudTime = 0;

ros::Publisher *pubLaserCloudPointer = NULL;
cv_bridge::CvImageConstPtr segImageCv;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  odomIDPointer = (odomIDPointer + 1) % stackNum;
  odomTimeStack[odomIDPointer] = odom->header.stamp.toSec();
  lidarXStack[odomIDPointer] = odom->pose.pose.position.x;
  lidarYStack[odomIDPointer] = odom->pose.pose.position.y;
  lidarZStack[odomIDPointer] = odom->pose.pose.position.z;
  lidarRollStack[odomIDPointer] = roll;
  lidarPitchStack[odomIDPointer] = pitch;
  lidarYawStack[odomIDPointer] = yaw;
}

void semImageHandler(const sensor_msgs::ImageConstPtr& image)
{
  imageTime = image->header.stamp.toSec();
  segImageCv = cv_bridge::toCvShare(image, "bgr8");

  imageInit = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  laserCloudTime = laserCloudIn->header.stamp.toSec();

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  newLaserCloud = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semanticScanGeneration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 50, odomHandler);

  ros::Subscriber subSegImage = nh.subscribe<sensor_msgs::Image> ("/camera/semantic_image", 2, semImageHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/registered_scan", 2, laserCloudHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/semantic_scan", 2);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (imageInit && newLaserCloud) {
      newLaserCloud = false;

      int laserCloudSize = laserCloud->points.size();
      if (laserCloudSize <= 0) continue;

      if (odomIDPointer < 0) continue;
      while (odomTimeStack[imageIDPointer] < imageTime - 0.001 &&
             imageIDPointer != (odomIDPointer + 1) % stackNum) {
        imageIDPointer = (imageIDPointer + 1) % stackNum;
      }
      if (fabs(odomTimeStack[imageIDPointer] - imageTime) > 0.001) continue;

      float lidarX = lidarXStack[imageIDPointer];
      float lidarY = lidarYStack[imageIDPointer];
      float lidarZ = lidarZStack[imageIDPointer];
      float lidarRoll = lidarRollStack[imageIDPointer];
      float lidarPitch = lidarPitchStack[imageIDPointer];
      float lidarYaw = lidarYawStack[imageIDPointer];

      int imageWidth = segImageCv->image.size().width;
      int imageHeight = segImageCv->image.size().height;

      float sinLidarRoll = sin(lidarRoll);
      float cosLidarRoll = cos(lidarRoll);
      float sinLidarPitch = sin(lidarPitch);
      float cosLidarPitch = cos(lidarPitch);
      float sinLidarYaw = sin(lidarYaw);
      float cosLidarYaw = cos(lidarYaw);

      pcl::PointXYZRGB point;
      laserCloudSeg->clear();
      for (int i = 0; i < laserCloudSize; i++) {
        float x1 = laserCloud->points[i].x - lidarX;
        float y1 = laserCloud->points[i].y - lidarY;
        float z1 = laserCloud->points[i].z - lidarZ;

        float x2 = x1 * cosLidarYaw + y1 * sinLidarYaw;
        float y2 = -x1 * sinLidarYaw + y1 * cosLidarYaw;
        float z2 = z1;

        float x3 = x2 * cosLidarPitch - z2 * sinLidarPitch;
        float y3 = y2;
        float z3 = x2 * sinLidarPitch + z2 * cosLidarPitch;

        float x4 = x3;
        float y4 = y3 * cosLidarRoll + z3 * sinLidarRoll;
        float z4 = -y3 * sinLidarRoll + z3 * cosLidarRoll - cameraOffsetZ;

        float horiDis = sqrt(x4 * x4 + y4 * y4);
        int horiPixelID = -imageWidth / (2 * PI) * atan2(y4, x4) + imageWidth / 2 + 1;
        int vertPixelID = -imageWidth / (2 * PI) * atan(z4 / horiDis) + imageHeight / 2 + 1;
        int pixelID = imageWidth * vertPixelID + horiPixelID;

        if (horiPixelID >= 0 && horiPixelID < imageWidth && vertPixelID >= 0 && vertPixelID < imageHeight) {
          point.x = laserCloud->points[i].x;
          point.y = laserCloud->points[i].y;
          point.z = laserCloud->points[i].z;
          point.b = segImageCv->image.data[3 * pixelID];
          point.g = segImageCv->image.data[3 * pixelID + 1];
          point.r = segImageCv->image.data[3 * pixelID + 2];

          laserCloudSeg->push_back(point);
        }
      }

      sensor_msgs::PointCloud2 laserCloudOut;
      pcl::toROSMsg(*laserCloudSeg, laserCloudOut);
      laserCloudOut.header.stamp = ros::Time().fromSec(laserCloudTime);
      laserCloudOut.header.frame_id = "map";
      pubLaserCloudPointer->publish(laserCloudOut);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
