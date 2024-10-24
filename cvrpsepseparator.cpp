#include "cvrpsepseparator.h"

CVRPSEPSeparator::CVRPSEPSeparator(VRPInstance &instance, Params &params)
    : instance(instance), params(params) {
    NoOfCustomers = instance.n - 1;
    CAP = instance.capacity;
    EpsForIntegrality = 1e-6;
    MaxNoOfCapCuts = 2;
    MaxNoOfFCITreeNodes = 20000;
    MaxNoOfFCICuts = 10;
    MaxNoOfMStarCuts = 30;
    MaxNoOfCombCuts = 20;
    MaxNoOfHypoCuts = 10;
    time = 0.0;
    nCuts = 0;

    // initialize Constraint structure
    CMGR_CreateCMgr(&MyCutsCMP, 100);
    CMGR_CreateCMgr(&MyOldCutsCMP, 100);

    // populate Demand vector
    int demandSum = 0;
    Demand = new int[NoOfCustomers + 2];
    for (int i = 0; i < instance.n; i++) {
        Demand[i] = instance.demands[i];
        demandSum += instance.demands[i];
    }

    QMin = demandSum - (instance.k - 1) * instance.capacity;
}

CVRPSEPSeparator::~CVRPSEPSeparator() {
    CMGR_FreeMemCMgr(&MyCutsCMP);
    CMGR_FreeMemCMgr(&MyOldCutsCMP);
    delete[] Demand;
}

CnstrMgrPointer CVRPSEPSeparator::getMyOldCutsPointer() { return MyOldCutsCMP; }

void CVRPSEPSeparator::setMaxNoOfCapCuts(int maxNoOfCuts) {
    MaxNoOfCapCuts = maxNoOfCuts;
}

// CVRPSEP uses `N` to refer to the depot.
int CVRPSEPSeparator::checkForDepot(int i) { return (i == instance.n) ? 0 : i; }

// Add capacity cuts.
int CVRPSEPSeparator::addCapacityCuts(int cutIndex,
                                      std::vector<std::vector<double>> &xValue,
                                      std::vector<CutData> &separatedCuts) {
    double RHS, LHS;
    std::vector<int> customers;
    int List[NoOfCustomers + 1];

    // Get customers in the cut.
    for (int i = 1; i <= MyCutsCMP->CPL[cutIndex]->IntListSize; i++) {
        customers.push_back(
            checkForDepot(MyCutsCMP->CPL[cutIndex]->IntList[i]));
    }
    std::sort(customers.begin(), customers.end());

    // Create the cut for x(S:S) <= (|S| - k(S))
    separatedCuts.emplace_back(CutData());
    CutData *cutData = &separatedCuts.back();
    cutData->RHS = MyCutsCMP->CPL[cutIndex]->RHS;
    cutData->sense = '<';
    cutData->name = "CVRPSEPCut";
    cutData->LHS = 0;

    for (int i = 0; i < customers.size(); i++) {
        for (int j = i + 1; j < customers.size(); j++) {
            cutData->coefs.push_back(
                std::make_tuple(customers[i], customers[j], 1.0));
            cutData->LHS += xValue[customers[i]][customers[j]];
        }
    }

    int nSepCuts = 1;
    int nVehicles = -cutData->RHS + customers.size();
    if (cutData->LHS <= cutData->RHS + EpsForIntegrality) {
        nSepCuts--;
        cutData = NULL;
        separatedCuts.pop_back();
    }
    nCuts += nSepCuts;

    return nSepCuts;
}

int CVRPSEPSeparator::separateCVRPSEPCuts(
    std::vector<std::vector<double>> &xValue,
    std::vector<CutData> &separatedCuts) {
    auto started = std::chrono::high_resolution_clock::now();
    // Count number of edges x_e > 0.
    int nEdges = 0;
    for (int i = 0; i < instance.n; i++) {
        for (int j = i + 1; j < instance.n; j++) {
            if (xValue[i][j] >= EpsForIntegrality) {
                nEdges++;
            }
        }
    }

    // Populate EdgeTail, EdgeHead and EdgeX.
    int *EdgeTail, *EdgeHead, k = 1;
    double *EdgeX;

    EdgeTail = new int[nEdges + 1];
    EdgeHead = new int[nEdges + 1];
    EdgeX = new double[nEdges + 1];

    for (int i = 0; i < instance.n; i++) {
        for (int j = i + 1; j < instance.n; j++) {
            if (xValue[i][j] >= EpsForIntegrality) {
                int u = (i == 0) ? instance.n : i;
                int v = j;

                EdgeTail[k] = u;
                EdgeHead[k] = v;
                EdgeX[k] = xValue[i][j];
                k++;
            }
        }
    }

    // Call CVRPSEP to separate capacity cuts.
    MaxCapViolation = 0;
    CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nEdges, EdgeTail,
                           EdgeHead, EdgeX, MyOldCutsCMP, MaxNoOfCapCuts,
                           EpsForIntegrality, &IntegerAndFeasible,
                           &MaxCapViolation, MyCutsCMP);

    // Free edges arrays.
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    // No cuts found.
    if (IntegerAndFeasible || MyCutsCMP->Size == 0) {
        auto done = std::chrono::high_resolution_clock::now();
        time += double(std::chrono::duration_cast<std::chrono::milliseconds>(
                           done - started)
                           .count()) /
                1000.0;

        return 0;
    }

    // Add separated capacity cuts to the input list of cuts.
    int nSepCuts = 0;
    for (int i = 0; i < MyCutsCMP->Size; i++) {
        if (MyCutsCMP->CPL[i]->CType == CMGR_CT_CAP) {
            nSepCuts += addCapacityCuts(i, xValue, separatedCuts);
        }
    }

    // Move the new cuts to the list of old cuts and reset size.
    for (int i = 0; i < MyCutsCMP->Size; i++) {
        CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
    }
    MyCutsCMP->Size = 0;

    auto done = std::chrono::high_resolution_clock::now();
    time += double(std::chrono::duration_cast<std::chrono::milliseconds>(
                       done - started)
                       .count()) /
            1000.0;

    return nSepCuts;
}
