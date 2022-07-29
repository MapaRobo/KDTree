#include <iostream>
#include <vector>
 
#include "subModules.h"

#define PI 3.14159265359

// TODO 
// 1. Secondary Pathing add in path in correct direction - same logic as before
// 2. Goal Intersection - More intersection Logic
// 3. Adding in yaw to predetermined path - Points towards next goal

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