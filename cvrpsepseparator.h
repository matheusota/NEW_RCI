#ifndef CVRPSEPSEPARATOR_H
#define CVRPSEPSEPARATOR_H
// #define SCIP_DEBUG
#include "params.h"
#include <cassert>
#include <lemon/list_graph.h>
#include <list>
#include <utility>
#include <ctime> // For CLOCKS_PER_SEC
#include <chrono>
#include "vrpinstance.h"
#include "params.h"
#include "cnstrmgr.h"
#include "capsep.h"
#include "mstarsep.h"
#include "fcisep.h"
#include "combsep.h"
#include "htoursep.h"
#include "cnstrmgr.h"
#include "cutdata.h"

class CVRPSEPSeparator {
  public:
    int nCuts;
    double time;

    CVRPSEPSeparator(VRPInstance &instance, Params &params);
    ~CVRPSEPSeparator();

    CnstrMgrPointer getMyOldCutsPointer();
    void setMaxNoOfCapCuts(int maxNoOfCuts);

    int separateCVRPSEPCuts(std::vector<std::vector<double>> &xValue,
                            std::vector<CutData> &separatedCuts);

  private:
    VRPInstance &instance;
    Params &params;

    // Parameters for the CVRPSEP package.
    int *Demand;
    CnstrMgrPointer MyCutsCMP, MyOldCutsCMP;
    double EpsForIntegrality, MaxCapViolation, MaxMStarViolation,
        MaxFCIViolation, MaxCombViolation, MaxHypoViolation;
    int NoOfCustomers, CAP, NoOfEdges, MaxNoOfCapCuts, MaxNoOfMStarCuts,
        MaxNoOfFCICuts, MaxNoOfCombCuts, MaxNoOfHypoCuts;
    char IntegerAndFeasible;
    int MaxNoOfFCITreeNodes;
    int QMin;

    int checkForDepot(int i);

    int addCapacityCuts(int cutIndex, std::vector<std::vector<double>> &xValue,
                        std::vector<CutData> &separatedCuts);
};

#endif // CVRPSEPSEPARATOR_H
