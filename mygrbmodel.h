#ifndef MYGRBMODEL_H
#define MYGRBMODEL_H

#include <gurobi_c++.h>
#include <utility>
#include <map>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <stack>
#include <chrono>
#include "vrpinstance.h"
#include "vrpsolution.h"
#include "params.h"
#include "cutdata.h"
#include "cvrpsepseparator.h"
#include "mygrbcallback.h"

class MyGRBModel {
  public:
    MyGRBModel(VRPInstance &instance, Params &params);
    ~MyGRBModel();

    void solve(VRPSolution &solution);

  private:
    VRPInstance &instance;
    Params &params;

    // Gurobi stuff.
    GRBEnv env;
    GRBModel model;
    std::vector<std::vector<GRBVar>> x;
    MyGRBCallback *callback;
    CVRPSEPSeparator *cvrpsepSeparator;

    void setBasicModel();
    void solveRootLP();
    void setSolution(VRPSolution &solution);
    bool addSeparatedCuts(std::vector<CutData> &separatedCuts);
};

#endif // MYGRBMODEL_H