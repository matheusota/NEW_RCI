#ifndef VRPSOLUTION_H
#define VRPSOLUTION_H

#include <iostream>
#include <vector>
#include "vrpinstance.h"

class VRPSolution {
  public:
    VRPInstance &instance;
    std::vector<std::vector<int>> routes;
    double cost;
    double rootBound;
    int time;
    int cvrpCuts;
    int nodesExplored;
    double rootGap;
    double rootTime;

    VRPSolution(VRPInstance &instance);
    void setRoutesFromSolution(std::vector<std::vector<int>> &edgeCount);
    void print();
};
#endif // VRPSOLUTION_H
