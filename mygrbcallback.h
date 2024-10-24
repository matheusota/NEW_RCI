#ifndef MYGRBCALLBACK_H
#define MYGRBCALLBACK_H

#include <gurobi_c++.h>
#include <cassert>
#include <list>
#include <utility>
#include <ctime> // For CLOCKS_PER_SEC
#include <chrono>
#include "vrpinstance.h"
#include "params.h"
#include "cvrpsepseparator.h"
#include "cutdata.h"

class MyGRBCallback : public GRBCallback {
  public:
    double rootBound;

    MyGRBCallback(VRPInstance &instance, Params &params, GRBModel &model,
                  std::vector<std::vector<GRBVar>> &x,
                  CVRPSEPSeparator *cvrpsepSeparator);
    void callback();

  private:
    double (GRBCallback::*solutionValue)(GRBVar);

    VRPInstance &instance;
    Params &params;
    GRBModel &model;
    std::vector<std::vector<GRBVar>> &x;
    CVRPSEPSeparator *cvrpsepSeparator;
    int prevNodeCount;
};

#endif // MYGRBCALLBACK_H
