#include <vector>
#include <iostream>
#include <cmath>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include "bits/stdc++.h"

#include "KDTree.hpp"
#include "treeWrapper.h"

#define PI 3.14159265359

KDTree constructkdtree(std::vector<std::vector<double>> points)
{
  pointVec pointsVec;
  point_t pt;
  for (int i = 0; i < points.size(); i++)
  {
    pt = {points[i][0], points[i][1]};
    pointsVec.push_back(pt);
  }
  KDTree tree(pointsVec);
  return tree;
}

std::vector<double> nnsearch(KDTree tree, std::vector<double> point)
{
  point_t pt;
  pt = {point[0], point[1]};
  auto res = tree.nearest_point(pt);
  std::vector<double> nearest(2);
  nearest[0] = res[0];
  nearest[1] = res[1];
  return nearest;
}

std::vector<std::vector<double>> KDTreeToPath(KDTree tree, double radius)
{
  double meanx = 0;
  double meany = 0;
  point_t pt = {.0, .0};
  auto pointsList = tree.neighborhood_points(pt, 100);

  for (int i = 0; i < pointsList.size(); i++)
  {
    meanx += pointsList[i][0];
    meany += pointsList[i][1];
  }
  meanx /= pointsList.size();
  meany /= pointsList.size();

  std::vector<std::vector<double>> path;
  double center[2] = {meanx, meany};
  double startdist = 16;

  for (double rad = 0; rad < 2 * PI; rad += PI / 48)
  {
    for (double dist = startdist; dist > 0; dist -= 0.01)
    {
      std::vector<double> point(2);
      std::vector<double> closestPoint(2);
      point[0] = center[0] + dist * cos(rad);
      point[1] = center[1] + dist * sin(rad);
      closestPoint = nnsearch(tree, point);
      double dist2 = distanceSquared(point, closestPoint);
      if (dist2 < radius * radius)
      {
        path.push_back(point);
        break;
      }
    }
  }
  return path;
}

double distanceSquared(std::vector<double> point1, std::vector<double> point2)
{
  double delx = point1[0] - point2[0];
  double dely = point1[1] - point2[1];
  double squaredDistance = delx * delx + dely * dely;
  return squaredDistance;
}

void plotVec(std::vector<std::vector<double>> vec, std::string str)
{
  std::ofstream file(str);
  file << "#x y" << std::endl;
  for (int i = 0; i < vec.size(); i++)
  {
    file << vec[i][0] << ' ' << vec[i][1] << std::endl;
  }
  file.close();
}

std::vector<std::vector<double>> straightLine(const std::vector<double> point1, const std::vector<double> point2)
{

  std::vector<std::vector<double>> vecOut;
  double x1 = point1[0];
  double y1 = point1[1];
  double x2 = point2[0];
  double y2 = point2[1];

  for (double i = 0; i <= 1; i += 0.01)
  {

    std::vector<double> vecTemp(2);
    vecTemp[0] = x1 + (x2 - x1) * i;
    vecTemp[1] = y1 + (y2 - y1) * i;
    vecOut.push_back(vecTemp);
  }
  return vecOut;
}

std::vector<std::vector<double>> minMaxObject(std::vector<std::vector<double>> path, std::vector<double> loc)
{

  double minAngle = atan2(path[0], loc);
  double maxAngle = atan2(path[0], loc);
  double minIndex = 0;
  double maxIndex = 0;

  double offset = 0;
  double del = 0;
  double angle = maxAngle;
  double prev = 0;

  for (int i = 0; i < path.size(); i++)
  {
    prev = angle;
    angle = atan2(path[i], loc);
    del = angle - prev;

    if ((del) > 3)
    {
      offset = -2 * 3.14159265359;
      angle += offset;
    }
    if (del < -3)
    {
      offset = 2 * 3.14159265359;
      angle += offset;
    }
    if (angle > maxAngle)
    {
      maxAngle = angle;
      maxIndex = i;
    }
    if (angle < minAngle)
    {
      minAngle = angle;
      minIndex = i;
    }
    std::cout << angle << ' ' << path[i][0] << ' ' << path[i][1] << ' ' << abs(del) << '\n';
  }
  std::vector<std::vector<double>> mmObj(2);
  mmObj[0] = path[minIndex];
  mmObj[1] = path[maxIndex];
  return mmObj;
}

bool lineSegSide(std::vector<double> point1, std::vector<double> point2, std::vector<double> point, std::vector<double> origin)
{
  double value = (point[0] - point1[0]) * (point2[1] - point1[1]) - (point[1] - point1[1]) * (point2[0] - point1[0]);
  double valueO = (origin[0] - point1[0]) * (point2[1] - point1[1]) - (origin[1] - point1[1]) * (point2[0] - point1[0]);
  return !(signbit(value) ^ signbit(valueO));
}

std::vector<std::vector<double>> sidePathView(std::vector<std::vector<double>> path, std::vector<double> loc, std::vector<std::vector<double>> mmCloud)
{
  bool prevValue = lineSegSide(mmCloud[0], mmCloud[1], path[0], loc);
  int startIndex = 0;
  int endIndex = path.size() - 1;
  for (int i = 0; i < path.size(); i++)
  {
    bool currentValue = lineSegSide(mmCloud[0], mmCloud[1], path[i], loc);
    if (currentValue == true && prevValue == false)
      startIndex = i;
    if (currentValue == false && prevValue == true)
      endIndex = i;
    prevValue = currentValue;
  }

  bool jump = false;
  if (startIndex > endIndex)
    jump = true;
  if (jump)
  {
    std::vector<std::vector<double>> start = slicing(path, startIndex, path.size() - 1);
    std::vector<std::vector<double>> end = slicing(path, 0, endIndex);
    std::vector<std::vector<double>> sidePath = concatenate(start, end);
    return sidePath;
  }
  std::vector<std::vector<double>> sidePath = slicing(path, startIndex, endIndex);
  return sidePath;
}

std::vector<std::vector<double>> slicing(std::vector<std::vector<double>> &arr, int X, int Y) {
  // Starting and Ending iterators
  auto start = arr.begin() + X;
  auto end = arr.begin() + Y + 1;
  // To store the sliced vector
  std::vector<std::vector<double>> result(Y - X + 1);
  // Copy vector using copy function()
  copy(start, end, result.begin());
  // Return the final sliced vector
  return result;
}

std::vector<std::vector<double>> concatenate(std::vector<std::vector<double>> &A, std::vector<std::vector<double>> &B)
{
  std::vector<std::vector<double>> AB;
  AB.reserve(A.size() + B.size()); // preallocate memory
  AB.insert(AB.end(), A.begin(), A.end());
  AB.insert(AB.end(), B.begin(), B.end());
  return AB;
}

bool ccw(std::vector<double> A, std::vector<double> B, std::vector<double> C)
{
  return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0]);
}

// Return true if line segments AB and CD intersect
bool intersect(std::vector<double> A, std::vector<double> B, std::vector<double> C, std::vector<double> D)
{
  return ccw(A, C, D) != ccw(B, C, D) && ccw(A, B, C) != ccw(A, B, D);
}

std::vector<double> pathGoalIntersect1(KDTree tree, std::vector<double> startGoal, std::vector<double> endGoal, double rad)
{
  std::vector<double> PathIntersect = startGoal;
  std::vector<double> slope{endGoal[0] - startGoal[0], endGoal[1] - startGoal[1]};

  for (double i = 0; i < 1; i += 0.01)
  {
    PathIntersect[0] += 0.01 * slope[0];
    PathIntersect[1] += 0.01 * slope[1];
    std::vector<double> closest = nnsearch(tree, PathIntersect);
    double dist = sqrt(distanceSquared(closest, PathIntersect));
    if (dist <= rad)
      break;
  }
  return PathIntersect;
}

std::vector<double> repeatPathLoop(KDTree tree, std::vector<std::vector<double>> Object, std::vector<std::vector<double>> Entirepath, const std::vector<double> startLocation, double rad, const std::vector<double> startGoal, const std::vector<double> endGoal, bool CW)
{
  bool done = false;
  std::vector<double> loc = startLocation;
  int goalIndex;
  std::vector<std::vector<double>> sidePath;

  for (int i = 1; i < 50; i++)
  {
    std::vector<std::vector<double>> mmObj = minMaxObject(Object, loc);
    std::vector<std::vector<double>> conevec1 = straightLine(mmObj[0], loc);
    std::vector<std::vector<double>> conevec2 = straightLine(mmObj[1], loc);
    sidePath = sidePathView(Entirepath, loc, mmObj);
    sidePath = pathSideChoice(sidePath, loc, CW);
    int goalIndex = goalIntersectIndex(sidePath, startGoal, endGoal);

    std::string s = std::to_string(i);

    plotVec(conevec1, "coneleft_" + s + ".dat");
    plotVec(conevec2, "coneright_" + s + ".dat");
    loc = sidePath[sidePath.size() - 1];
    
    if (goalIndex != -1) {
      std::vector<std::vector<double>> lastPath = slicing(sidePath, 0, goalIndex);
      plotVec(lastPath, "path_fin.dat");
      break;
    }

    plotVec(sidePath, "path_" + s + ".dat");
  }
  return loc;
}

double atan2(std::vector<double> point, std::vector<double> pos)
{
  return atan2(point[1] - pos[1], point[0] - pos[0]);
}

std::vector<std::vector<double>> FindInitialPath(std::vector<std::vector<double>> loopPath, std::vector<double> startNode, std::vector<double> endNode, std::vector<std::vector<double>> object, bool CW)
{
  std::vector<std::vector<double>> mmObj = minMaxObject(object, startNode);
  std::vector<std::vector<double>> sidePath = sidePathView(loopPath, startNode, mmObj);

  int intersectIndex = 0;
  for (int i = 0; i < sidePath.size() - 1; i++)
  {
    if (intersect(sidePath[i], sidePath[i + 1], startNode, endNode)) {
      intersectIndex = i;
    }
  }

  std::vector<std::vector<double>> goalToObjPath = straightLine(startNode, sidePath[intersectIndex]);
  std::vector<std::vector<double>> initialObjPath;
  if (CW) {
    intersectIndex -= 2;
    initialObjPath = slicing(sidePath, 0, intersectIndex);
    initialObjPath = reverseVec(initialObjPath);
  }
  else initialObjPath = slicing(sidePath, intersectIndex, sidePath.size() - 1);
  std::vector<std::vector<double>> fullPath = concatenate(goalToObjPath, initialObjPath);
  return fullPath;
}

std::vector<std::vector<double>> reverseVec(std::vector<std::vector<double>> vec) {
  std::vector<std::vector<double>> vecOut(vec.size());
  for (int i = 0; i <= vec.size() / 2; i++) {
    vecOut[i] = vec[vec.size() - 1 - i]; 
    vecOut[vec.size() - 1 - i] = vec[i];
  }
  return vecOut;
}

std::vector<std::vector<double>> pathSideChoice(std::vector<std::vector<double>> path, std::vector<double> loc, bool CW) {
  double minManhattenDistance = 1000; // Computationally easier than eulidean and shouldn't matter bit picture
  int minindex = 0;
  for (int i = 0; i < path.size(); i++) {
    double dist = abs((path[i][0] - loc[0]) + (path[i][1] - loc[1]));
    if (minManhattenDistance > dist) {
      minindex = i;
      minManhattenDistance = dist;
    }
  } 
  
  std::vector<std::vector<double>> directedPath;

  if (CW) {
    directedPath = slicing(path, 0, minindex);
    directedPath = reverseVec(directedPath);
  }
  else {
    directedPath = slicing(path, minindex, path.size() - 1);
  }
  return directedPath;
}

int goalIntersectIndex(std::vector<std::vector<double>> path, std::vector<double> start, std::vector<double> goal) {
  for (int i = 0; i < path.size() - 1; i++) {
    if (intersect(path[i], path[i + 1], start, goal)) return i;
  }
  return -1;
}

std::vector<double> pathToYaw(std::vector<std::vector<double>> path) {
  std::vector<double> yawVec(path.size());
  for (int i = 0; i < path.size() - 1; i++) {
    std::vector<double> slope {path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]};
    double yaw = atan2(slope[1], slope[0]);
    yawVec[i] = yaw;
  }
  yawVec[yawVec.size() - 1] = yawVec[yawVec.size() - 2];
  return yawVec;
}