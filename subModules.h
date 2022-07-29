
#include <vector>


std::vector<std::vector<double>> createObstacle();

void obstacleAvoidance(std::vector<std::vector<double>> object, double rad, std::vector<double> startGoal, std::vector<double> endGoal, bool CW, std::vector<double> initialPos);