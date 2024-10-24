#include "mygrbcallback.h"

MyGRBCallback::MyGRBCallback(VRPInstance &instance, Params &params,
                             GRBModel &model,
                             std::vector<std::vector<GRBVar>> &x,
                             CVRPSEPSeparator *cvrpsepSeparator)
    : instance(instance), params(params), model(model), x(x) {
    this->cvrpsepSeparator = cvrpsepSeparator;
    prevNodeCount = -1;
    rootBound = 1e9;
}

void MyGRBCallback::callback() {
    try {
        int currNodeCount;
        double currSolCost = 0.0;
        double currDualBound = 0.0;
        bool isMIPSol = (where == GRB_CB_MIPSOL);
        bool isMIPNode = (where == GRB_CB_MIPNODE) &&
                         (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL);

        if (!isMIPSol && !isMIPNode) {
            return;
        }

        // Get solution values according to solution type.
        // Integer solution.
        if (isMIPSol) {
            currNodeCount = (int)getDoubleInfo(GRB_CB_MIPSOL_NODCNT);
            solutionValue = &MyGRBCallback::getSolution;
            currSolCost = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            currDualBound = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
        }
        // Fractional solution.
        else {
            assert(isMIPNode);
            currNodeCount = (int)getDoubleInfo(GRB_CB_MIPNODE_NODCNT);
            solutionValue = &MyGRBCallback::getNodeRel;
            currDualBound = getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
        }

        if (currNodeCount == 0) {
            rootBound = currDualBound;
        }
        prevNodeCount = currNodeCount;

        // Construct xValue.
        std::vector<std::vector<double>> xValue(
            instance.n, std::vector<double>(instance.n));
        for (int i = 0; i < instance.n; i++) {
            for (int j = i + 1; j < instance.n; j++) {
                xValue[i][j] = (this->*solutionValue)(x[i][j]);
            }
        }

        // Call separation routines.
        std::vector<CutData> separatedCuts;
        cvrpsepSeparator->separateCVRPSEPCuts(xValue, separatedCuts);

        // Add cuts.
        for (CutData &cutData : separatedCuts) {
            double sign = (cutData.sense == GRB_LESS_EQUAL) ? 1 : -1;
            if (sign * (cutData.LHS - cutData.RHS) <= 1e-6) {
                continue;
            }

            GRBLinExpr expr = 0.0;
            for (const auto &[i, j, coef] : cutData.coefs) {
                expr += coef * x[i][j];
            }

            addLazy(expr, cutData.sense, cutData.RHS);
        }
    } catch (GRBException e) {
        std::cout << "MyGRBCallback: Error when calling callback. Error code: "
                  << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
        exit(1);
    } catch (...) {
        std::cout << "Unknown error!" << std::endl;
        exit(1);
    }
}