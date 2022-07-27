#include <iostream>
#include <cmath>
 
#include "treeWrapper.h"

#define PI 3.14159265359

// TODO 
// 1. Secondary Pathing add in path in correct direction
// 2. Goal Intersection
// 3. Adding in yaw to predetermined path

// Edge Cases
// 1. Goal covered by object 


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

    KDTree tree = constructkdtree(object);
    std::vector<std::vector<double>> path = KDTreeToPath(tree, rad);
    std::vector<std::vector<double>> initialPath = InitalPathMerge(path, startGoal, endGoal, object);
    loc = initialPath[initialPath.size() - 1];
    repeatPathLoop(tree, object, path, loc, rad, startGoal, endGoal);
    //std::vector<double> intersect = pathGoalIntersect1(tree, startGoal, endGoal, rad);
    //loc = intersect;
    //std::vector<double> end = repeatPathLoop(tree, object, path, loc, rad, startGoal, endGoal);
 
    //std::vector<std::vector<double>> goalLine = straightLine(startGoal, intersect);
    
    plotVec(object, "object.dat");
    plotVec(initialPath, "path.dat");
  
}