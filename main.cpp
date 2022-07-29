#include <iostream>
#include <vector>
 
#include "subModules.h"

#define PI 3.14159265359

// TODO 
// 1. Edge Cases

// Edge Cases
// 1. Goal covered by object 


int main() {
    double rad = 1;
    bool CW = false;
    std::vector<double> loc{5, 0};
    std::vector<double> startGoal{6, 0};
    std::vector<double> endGoal{-6, 0};

    std::vector<std::vector<double>> obstacle = createObstacle();
    obstacleAvoidance(obstacle, rad, startGoal, endGoal, CW, loc);
}

// To run:
// g++ main.cpp subModules.cpp treeWrapper.cpp KDTree.cpp -o localPlanner.o && ./localPlanner.o
// Then in gnuplot run the code in gnuplot.plt