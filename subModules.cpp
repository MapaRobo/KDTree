#include <iostream>
#include <cmath>
#include <vector>
 
#include "treeWrapper.h"

#define PI 3.14159265359

std::vector<std::vector<double>> createObstacle() {
    double radius = 1;
    std::vector<std::vector<double>> object;
    for (double i = 0; i < 4 * PI; i += PI / 24) {
        std::vector<double> point(2);
        point[0] = radius * sin(i);
        point[1] = radius * cos(1.5 * i);
        object.push_back(point);
    }
    return object;
}
void obstacleAvoidance(std::vector<std::vector<double>> object, double rad, std::vector<double> startGoal, std::vector<double> endGoal, bool CW, std::vector<double> initialPos) {
    // Initial Move with larger field of view
    
    std::vector<double> loc = initialPos;
    KDTree tree = constructkdtree(object);
    std::vector<std::vector<double>> path = KDTreeToPath(tree, rad);
    std::vector<std::vector<double>> initialPath = FindInitialPath(path, startGoal, endGoal, object, CW);
    
    
    loc = initialPath[initialPath.size() - 1];

    // Move until path intersects with goal line
    repeatPathLoop(tree, object, path, loc, rad, startGoal, endGoal, CW);


    plotVec(object, "object.dat");
    plotVec(initialPath, "path.dat");
}
