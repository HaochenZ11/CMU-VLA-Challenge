#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

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

string map_file_dir;
string object_list_file_dir;
int maxObjectNum = 100;

pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZI>());

void readMapFile()
{
  FILE* map_file = fopen(map_file_dir.c_str(), "r");
  if (map_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(map_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(map_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  mapCloud->clear();
  pcl::PointXYZI point;
  int val1, val2, val3, val4;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(map_file, "%f", &point.x);
    val2 = fscanf(map_file, "%f", &point.y);
    val3 = fscanf(map_file, "%f", &point.z);
    val4 = fscanf(map_file, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    mapCloud->push_back(point);
  }

  fclose(map_file);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bboxCalculation");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("map_file_dir", map_file_dir);
  nhPrivate.getParam("object_list_file_dir", object_list_file_dir);
  nhPrivate.getParam("maxObjectNum", maxObjectNum);

  readMapFile();
  int mapCloudSize = mapCloud->points.size();
  
  printf ("\nRead %d points from map file.\n", mapCloudSize);

  const int objNumConst = maxObjectNum;
  float objCX[objNumConst] = {0};
  float objCY[objNumConst] = {0};
  int objSize[objNumConst] = {0};

  for (int i = 0; i < mapCloudSize; i++) {
    int j = int(mapCloud->points[i].intensity);
    if (j < 0 || j >= maxObjectNum) continue;
    
    objCX[j] += mapCloud->points[i].x;
    objCY[j] += mapCloud->points[i].y;
    objSize[j]++;
  }
  
  for (int j = 0; j < maxObjectNum; j++) {
    if (objSize[j] > 0) {
      objCX[j] /= objSize[j];
      objCY[j] /= objSize[j];
    }
  }

  float objC11[objNumConst] = {0};
  float objC12[objNumConst] = {0};
  float objC22[objNumConst] = {0};

  for (int i = 0; i < mapCloudSize; i++) {
    int j = int(mapCloud->points[i].intensity);
    if (j < 0 || j >= maxObjectNum) continue;
    
    float dx = mapCloud->points[i].x - objCX[j];
    float dy = mapCloud->points[i].y - objCY[j];

    objC11[j] += dx * dx;
    objC12[j] += dx * dy;
    objC22[j] += dy * dy;
  }
  
  for (int j = 0; j < maxObjectNum; j++) {
    if (objSize[j] > 0) {
      objC11[j] /= objSize[j];
      objC12[j] /= objSize[j];
      objC22[j] /= objSize[j];
    }
  }
  
  cv::Mat matCov(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matE(1, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matV(2, 2, CV_32F, cv::Scalar::all(0));
  
  float objHeading[objNumConst] = {0};
  
  for (int j = 0; j < maxObjectNum; j++) {
    if (objSize[j] > 0) {
      matCov.at<float>(0, 0) = objC11[j];
      matCov.at<float>(0, 1) = objC12[j];
      matCov.at<float>(1, 0) = objC12[j];
      matCov.at<float>(1, 1) = objC22[j];
  
      cv::eigen(matCov, matE, matV);
      objHeading[j] = atan2(matV.at<float>(0, 1), matV.at<float>(0, 0));
    }
  }
  
  float objMinL[objNumConst] = {0};
  float objMaxL[objNumConst] = {0};
  float objMinW[objNumConst] = {0};
  float objMaxW[objNumConst] = {0};
  float objMinH[objNumConst] = {0};
  float objMaxH[objNumConst] = {0};

  for (int j = 0; j < maxObjectNum; j++) {
    if (objSize[j] > 0) {
      objMinL[j] = 1000000.0;
      objMaxL[j] = -1000000.0;
      objMinW[j] = 1000000.0;
      objMaxW[j] = -1000000.0;
      objMinH[j] = 1000000.0;
      objMaxH[j] = -1000000.0;
    }
  }

  for (int i = 0; i < mapCloudSize; i++) {
    int j = int(mapCloud->points[i].intensity);
    if (j < 0 || j >= maxObjectNum) continue;

    float l = cos(objHeading[j]) * mapCloud->points[i].x + sin(objHeading[j]) * mapCloud->points[i].y;
    float w = -sin(objHeading[j]) * mapCloud->points[i].x + cos(objHeading[j]) * mapCloud->points[i].y;
    float h = mapCloud->points[i].z;

    if (objMinL[j] > l) objMinL[j] = l;
    if (objMaxL[j] < l) objMaxL[j] = l;
    if (objMinW[j] > w) objMinW[j] = w;
    if (objMaxW[j] < w) objMaxW[j] = w;
    if (objMinH[j] > h) objMinH[j] = h;
    if (objMaxH[j] < h) objMaxH[j] = h;
  }

  FILE *object_list_file = fopen(object_list_file_dir.c_str(), "w");
  if (object_list_file == NULL) {
    printf ("\nCannot write output file, exit.\n\n");
    exit(1);
  }

  int objectCount = 0;
  for (int j = 0; j < maxObjectNum; j++) {
    if (objSize[j] > 0) {
      float midL = (objMinL[j] + objMaxL[j]) / 2.0;
      float midW = (objMinW[j] + objMaxW[j]) / 2.0;
      float midH = (objMinH[j] + objMaxH[j]) / 2.0;
      float midX = cos(objHeading[j]) * midL - sin(objHeading[j]) * midW;
      float midY = sin(objHeading[j]) * midL + cos(objHeading[j]) * midW;
  
      fprintf(object_list_file, "%d %f %f %f %f %f %f %f\n", j, midX, midY, midH, 
              objMaxL[j] - objMinL[j], objMaxW[j] - objMinW[j], objMaxH[j] - objMinH[j], objHeading[j]);
              
      objectCount++;
    }
  }

  fclose(object_list_file);

  printf ("\nBounding boxes of %d objects saved to file. Exit.\n", objectCount);

  return 0;
}
