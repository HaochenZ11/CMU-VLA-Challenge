#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string ref_traj_dir;
string actual_traj_dir;
double penaltyScale = 0.01;

pcl::PointCloud<pcl::PointXYZ>::Ptr refTraj(new pcl::PointCloud<pcl::PointXYZ>());

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scoreCalculation");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("ref_traj_dir", ref_traj_dir);
  nhPrivate.getParam("actual_traj_dir", actual_traj_dir);
  nhPrivate.getParam("penaltyScale", penaltyScale);

  pcl::PLYReader ply_reader;
  if (ply_reader.read(ref_traj_dir, *refTraj) == -1) {
    printf("\nCannot read reference trajectory file, exit.\n\n");
    exit(1);
  }

  FILE *actual_traj_file = fopen(actual_traj_dir.c_str(), "r");
  if (actual_traj_file == NULL) {
    printf ("\nCannot read actual trajectory file, exit.\n\n");
    exit(1);
  }


  float penalty = 0;
  int actualTrajSize = 0;
  int refTrajSize = refTraj->size();
  int val1, val2, val3, val4, val5, val6, val7;
  float x, y, z, roll, pitch, yaw, time, timeLast = 0;
  while (1) {
    val1 = fscanf(actual_traj_file, "%f", &x);
    val2 = fscanf(actual_traj_file, "%f", &y);
    val3 = fscanf(actual_traj_file, "%f", &z);
    val4 = fscanf(actual_traj_file, "%f", &roll);
    val5 = fscanf(actual_traj_file, "%f", &pitch);
    val6 = fscanf(actual_traj_file, "%f", &yaw);
    val7 = fscanf(actual_traj_file, "%f", &time);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) {
      break;
    }

    float minDelta = 1000000.0;
    for (int i = 0; i < refTrajSize; i++) {
      float deltaX = refTraj->points[i].x - x;
      float deltaY = refTraj->points[i].y - y;
      float delta = sqrt(deltaX * deltaX + deltaY * deltaY);
      
      if (minDelta > delta) minDelta = delta;
    }

    penalty += penaltyScale * minDelta * (time - timeLast);
    timeLast = time;
    actualTrajSize++;
  }
  
  printf ("\nRef traj points: %d, Actual traj points: %d, Penalty: %f.\n", refTrajSize, actualTrajSize, penalty);
  printf ("Pleae type in points.\n");

  int points = 0;
  scanf ("%d", &points);

  printf ("Score: %f.\n\n", points - penalty);

  return 0;
}
