#include <vector>
#include <string>

#include "KDTree.hpp"

KDTree constructkdtree(std::vector<std::vector<double>> points);

std::vector<double> nnsearch(KDTree tree, std::vector<double> point);

std::vector<std::vector<double>> KDTreeToPath(KDTree tree, double radius);

double distanceSquared(std::vector<double> point1, std::vector<double> point2);

void plotVec(std::vector<std::vector<double>> vec, std::string str);


std::vector<std::vector<double>> straightLine(std::vector<double> point1, std::vector<double> point2);

std::vector<std::vector<double>>  minMaxPath(std::vector<std::vector<double>> path, std::vector<double> pos);

std::vector<std::vector<double>> minMaxCloud(std::vector<std::vector<double>> minMaxPath, double radius, std::vector<double> loc);

bool lineSegSide(std::vector<double> point1, std::vector<double> point2, std::vector<double> point, std::vector<double> origin);

std::vector<std::vector<double>> sidePathView(std::vector<std::vector<double>> path, std::vector<double> loc, std::vector<std::vector<double>> mmCloud);

std::vector<std::vector<double>> slicing(std::vector<std::vector<double>>& arr,
                    int X, int Y);

std::vector<std::vector<double>> concatenate(std::vector<std::vector<double>>& A, std::vector<std::vector<double>>& B);

bool ccw(std::vector<double> A, std::vector<double> B, std::vector<double> C);

bool intersect(std::vector<double> A, std::vector<double> B, std::vector<double> C, std::vector<double> D);

std::vector<double> pathGoalIntersect(std::vector<std::vector<double>> path, std::vector<double> startGoal, std::vector<double> endGoal, double meanx, double meany);

std::vector<double> endLocalPath(KDTree tree, std::vector<std::vector<double>> Object, std::vector<std::vector<double>> Entirepath, std::vector<double> startLocation, 
                                double rad, std::vector<double> startGoal, std::vector<double> endGoal);

double atan2(std::vector<double> point, std::vector<double> pos);