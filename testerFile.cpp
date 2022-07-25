#include <iostream>
#include <cmath>
 
#include "treeWrapper.h"

#define PI 3.14159265359

int main() {
    double rad = 1;
    std::vector<double> loc{2, 0};
    std::vector<double> startGoal{12, 2};
    std::vector<double> endGoal{-12, -2};

    double radius = 1;
    std::vector<std::vector<double>> object;
    for (double i = 0; i < 2 * PI; i += PI / 24) {
        std::vector<double> point(2);
        point[0] = radius * sin(i);
        point[1] = radius * cos(i);
        object.push_back(point);
    }
    
    // std::vector<double> point1(2);
    // point1[0] = 0;
    // point1[1] = 0;
    // object.push_back(point1);

    // std::vector<double> point2(2);
    // point2[0] = 0;
    // point2[1] = 1;
    // object.push_back(point2);

    // std::vector<double> point3(2);
    // point3[0] = 1;
    // point3[1] = 1;
    // object.push_back(point3);

    // std::vector<double> point4(2);
    // point4[0] = 1;
    // point4[1] = 0;
    // object.push_back(point4);

    KDTree tree = constructkdtree(object);
    std::vector<std::vector<double>> path = KDTreeToPath(tree, rad);

    // std::vector<std::vector<double>> mmPath = minMaxPath(path, loc);
    // std::vector<std::vector<double>> mmCloud = minMaxCloud(mmPath, rad, loc);
    // std::vector<std::vector<double>> conevec1 = straightLine(mmCloud[0], loc);
    // std::vector<std::vector<double>> conevec2 = straightLine(mmCloud[1], loc);
    // std::vector<std::vector<double>> sidePath = sidePathView(path, loc, mmCloud);

    std::vector<double> end = endLocalPath(tree, object, path, loc, rad, startGoal, endGoal);
 
    std::vector<std::vector<double>> goalLine = straightLine(startGoal, endGoal);

    plotVec(object, "object.dat");
    plotVec(goalLine, "goalLine.dat");
  
}