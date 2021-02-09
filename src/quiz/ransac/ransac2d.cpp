/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cmath>
#include <unordered_set>

#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  struct Line {
    int pointIdx1, pointIdx2;
    float A, B, C;  // Line model is Ax + By + C = 0
    int inlierCount = 0;
  };

  int maxInlierCount = 0;

  Line bestLine;
  int pointsCount = cloud->points.size();

  float distanceToLine = 0.0f;

  for (int iter = 0; iter < maxIterations; ++iter) {
    Line currentPlane;

    currentPlane.pointIdx1 = rand() % (pointsCount - 1);
    currentPlane.pointIdx2 = rand() % (pointsCount - 1);
    while (currentPlane.pointIdx1 == currentPlane.pointIdx2) {
      currentPlane.pointIdx2 = rand() % (pointsCount - 1);
    }

    pcl::PointXYZ point1 = cloud->points[currentPlane.pointIdx1];
    pcl::PointXYZ point2 = cloud->points[currentPlane.pointIdx2];

    currentPlane.A = point1.y - point2.y;
    currentPlane.B = point2.x - point1.x;
    currentPlane.C = (point1.x * point2.y) - (point2.x * point1.y);

    for (int index = 0; index < cloud->points.size(); ++index) {
      pcl::PointXYZ point = cloud->points[index];
      distanceToLine = abs(currentPlane.A * point.x + currentPlane.B * point.y +
                           currentPlane.C) /
                       sqrt(currentPlane.A * currentPlane.A +
                            currentPlane.B * currentPlane.B);
      if (distanceToLine <= distanceTol) {
        ++currentPlane.inlierCount;
      }
    }

    if (currentPlane.inlierCount > maxInlierCount) {
      maxInlierCount = currentPlane.inlierCount;
      bestLine = currentPlane;
    }

    distanceToLine = 0.0f;
  }

  for (int index = 0; index < cloud->points.size(); ++index) {
    pcl::PointXYZ point = cloud->points[index];
    distanceToLine =
        abs(bestLine.A * point.x + bestLine.B * point.y + bestLine.C) /
        sqrt(bestLine.A * bestLine.A + bestLine.B * bestLine.B);
    if (distanceToLine <= distanceTol) {
      inliersResult.emplace(index);
    }
  }

  return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  struct Plane {
    int pointIdx1, pointIdx2, pointIdx3;
    float A, B, C, D;  // Plane model is Ax + By + Cz + D = 0
    int inlierCount = 0;
  };

  int maxInlierCount = 0;

  Plane bestPlane;
  int pointsCount = cloud->points.size();

  float distanceToPlane = 0.0f;

  for (int iter = 0; iter < maxIterations; ++iter) {
    Plane currentPlane;

    currentPlane.pointIdx1 = rand() % (pointsCount - 1);
    currentPlane.pointIdx2 = rand() % (pointsCount - 1);
    while (currentPlane.pointIdx2 == currentPlane.pointIdx1) {
      currentPlane.pointIdx2 = rand() % (pointsCount - 1);
    }
    currentPlane.pointIdx3 = rand() % (pointsCount - 1);
    while (currentPlane.pointIdx3 == currentPlane.pointIdx1 ||
           currentPlane.pointIdx3 == currentPlane.pointIdx2) {
      currentPlane.pointIdx3 = rand() % (pointsCount - 1);
    }

    pcl::PointXYZ point1 = cloud->points[currentPlane.pointIdx1];
    pcl::PointXYZ point2 = cloud->points[currentPlane.pointIdx2];
    pcl::PointXYZ point3 = cloud->points[currentPlane.pointIdx3];

    currentPlane.A = (point2.y - point1.y) * (point3.z - point1.z) -
                     (point2.z - point1.z) * (point3.y - point1.y);
    currentPlane.B = (point2.z - point1.z) * (point3.x - point1.x) -
                     (point2.x - point1.x) * (point3.z - point1.z);
    currentPlane.C = (point2.x - point1.x) * (point3.y - point1.y) -
                     (point2.y - point1.y) * (point3.x - point1.x);
    currentPlane.D =
        -1 * (currentPlane.A * point1.x + currentPlane.B * point1.y +
              currentPlane.C * point1.z);

    for (int index = 0; index < cloud->points.size(); ++index) {
      pcl::PointXYZ point = cloud->points[index];
      distanceToPlane =
          abs(currentPlane.A * point.x + currentPlane.B * point.y +
              currentPlane.C * point.z + currentPlane.D) /
          sqrt(currentPlane.A * currentPlane.A +
               currentPlane.B * currentPlane.B +
               currentPlane.C * currentPlane.C);
      if (distanceToPlane <= distanceTol) {
        ++currentPlane.inlierCount;
      }
    }

    if (currentPlane.inlierCount > maxInlierCount) {
      maxInlierCount = currentPlane.inlierCount;
      bestPlane = currentPlane;
    }

    distanceToPlane = 0.0f;
  }

  for (int index = 0; index < cloud->points.size(); ++index) {
    pcl::PointXYZ point = cloud->points[index];
    distanceToPlane =
        abs(bestPlane.A * point.x + bestPlane.B * point.y +
            bestPlane.C * point.z + bestPlane.D) /
        sqrt(bestPlane.A * bestPlane.A + bestPlane.B * bestPlane.B +
             bestPlane.C * bestPlane.C);
    if (distanceToPlane <= distanceTol) {
      inliersResult.emplace(index);
    }
  }

  return inliersResult;
}

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // std::unordered_set<int> inliers = Ransac(cloud, 20, 0.75f);
  std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.3f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
