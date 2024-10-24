#include "mygrbmodel.h"

MyGRBModel::MyGRBModel(VRPInstance &instance, Params &params)
    : instance(instance), params(params), model(env) {
    x.resize(instance.n, std::vector<GRBVar>(instance.n));

    // Set model.
    setBasicModel();

    // Set Gurobi callback and separator.
    cvrpsepSeparator = new CVRPSEPSeparator(instance, params);
    callback = new MyGRBCallback(instance, params, model, x, cvrpsepSeparator);
    model.setCallback(callback);
}

MyGRBModel::~MyGRBModel() {
    delete callback;
    delete cvrpsepSeparator;
}

void MyGRBModel::setBasicModel() {
    model.set(GRB_StringAttr_ModelName, "GurobiModel");
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Must set to use lazy contraints.
    model.getEnv().set(GRB_IntParam_LazyConstraints, 1);
    model.getEnv().set(GRB_IntParam_PreCrush, 1);

    // Some other parameters.
    model.getEnv().set(GRB_DoubleParam_TimeLimit, params.timeLimit);
    model.getEnv().set(GRB_IntParam_Threads, 4);
    model.getEnv().set(GRB_DoubleParam_MIPGap, 1e-6);

    // Set x variables.
    for (int i = 0; i < instance.n; i++) {
        for (int j = i + 1; j < instance.n; j++) {
            double UB = (i == 0) ? 2.0 : 1.0;

            x[i][j] = model.addVar(
                0.0, UB, instance.distances[i][j], GRB_INTEGER,
                "x_" + std::to_string(i) + "," + std::to_string(j));
        }
    }

    // Incidence constraints.
    for (int i = 0; i < instance.n; i++) {
        double RHS = (i == 0) ? 2.0 * instance.k : 2.0;

        GRBLinExpr expr = 0.0;
        for (int j = 0; j < instance.n; j++) {
            if (j == i) {
                continue;
            }

            int first = (i < j) ? i : j;
            int second = (i < j) ? j : i;
            expr += x[first][second];
        }
        model.addConstr(expr == RHS, "incidence_" + std::to_string(i));
    }
    model.update();
}

void MyGRBModel::solve(VRPSolution &solution) {
    // First we solve the root.
    auto started = std::chrono::high_resolution_clock::now();
    solveRootLP();
    auto done = std::chrono::high_resolution_clock::now();
    solution.rootTime =
        double(std::chrono::duration_cast<std::chrono::milliseconds>(done -
                                                                     started)
                   .count()) /
        1000.0;

    // Decrease time limit.
    double timeLimit = params.timeLimit;
    timeLimit -= solution.rootTime;
    if (timeLimit < 0) {
        return;
    }

    // Solve the MIP.
    model.getEnv().set(GRB_DoubleParam_TimeLimit, timeLimit);
    model.update();
    model.optimize();

    // Set solution.
    setSolution(solution);
}

void MyGRBModel::setSolution(VRPSolution &solution) {
    std::cout << "Gurobi finished running!" << std::endl;
    solution.cost = model.get(GRB_DoubleAttr_ObjVal);
    solution.nodesExplored = model.get(GRB_DoubleAttr_NodeCount);
    solution.rootBound = callback->rootBound;
    solution.rootGap = (solution.cost - solution.rootBound) / solution.cost;
    solution.cvrpCuts = cvrpsepSeparator->nCuts;

    // Get back the solution.
    if (solution.cost <= 1e6) {
        std::vector<std::vector<int>> edgeCount(instance.n,
                                                std::vector<int>(instance.n));
        for (int i = 0; i < instance.n; i++) {
            for (int j = i + 1; j < instance.n; j++) {
                edgeCount[i][j] =
                    int(std::round(x[i][j].get(GRB_DoubleAttr_X)));
                edgeCount[j][i] = edgeCount[i][j];

                if (edgeCount[i][j] > 0) {
                    std::cout << "x_" << i << "," << j << " = "
                              << edgeCount[i][j] << std::endl;
                }
            }
        }
        solution.setRoutesFromSolution(edgeCount);
    } else {
        std::cout << "Didn’t find optimal solution." << std::endl;
    }
}

void MyGRBModel::solveRootLP() {
    auto started = std::chrono::high_resolution_clock::now();

    // Change variables to continuous.
    for (int i = 0; i < instance.n; i++) {
        for (int j = i + 1; j < instance.n; j++) {
            x[i][j].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
        }
    }
    model.update();

    double prevDualBound = 0.0;
    int noImprovCount = 0;
    bool solveLP = true;
    std::vector<std::vector<double>> xValue(instance.n,
                                            std::vector<double>(instance.n));
    std::vector<CutData> separatedCuts;

    while (solveLP && noImprovCount <= 20) {
        solveLP = false;
        model.update();
        model.optimize();

        // Check time limit.
        auto done = std::chrono::high_resolution_clock::now();
        double elapsedTime =
            double(std::chrono::duration_cast<std::chrono::milliseconds>(
                       done - started)
                       .count()) /
            1000.0;
        if (elapsedTime > params.timeLimit) {
            return;
        }

        // Get dual bound and update no improvement counter.
        double currDualBound = model.get(GRB_DoubleAttr_ObjBound);
        if (currDualBound - prevDualBound >= 1e-3) {
            noImprovCount = 0;
        } else {
            noImprovCount++;
        }
        prevDualBound = currDualBound;

        // Get current solution.
        for (int i = 0; i < instance.n; i++) {
            for (int j = i + 1; j < instance.n; j++) {
                xValue[i][j] = x[i][j].get(GRB_DoubleAttr_X);
                xValue[j][i] = xValue[i][j];
            }
        }

        separatedCuts.clear();
        cvrpsepSeparator->separateCVRPSEPCuts(xValue, separatedCuts);

        if (addSeparatedCuts(separatedCuts)) {
            noImprovCount = 0;
            solveLP = true;
            continue;
        }
    }

    // Change variables back to integers.
    for (int i = 0; i < instance.n; i++) {
        for (int j = i + 1; j < instance.n; j++) {
            x[i][j].set(GRB_CharAttr_VType, GRB_INTEGER);
        }
    }
    model.update();
}

bool MyGRBModel::addSeparatedCuts(std::vector<CutData> &separatedCuts) {
    bool addedCut = false;

    for (CutData &cutData : separatedCuts) {
        double sign = (cutData.sense == GRB_LESS_EQUAL) ? 1 : -1;
        if (sign * (cutData.LHS - cutData.RHS) <= 1e-6) {
            continue;
        }

        GRBLinExpr expr = 0.0;
        for (const auto &[i, j, coef] : cutData.coefs) {
            expr += coef * x[i][j];
        }

        GRBConstr addedConstraint =
            model.addConstr(expr, cutData.sense, cutData.RHS, cutData.name);
        addedConstraint.set(GRB_IntAttr_Lazy, 3);
        addedCut = true;
    }

    return addedCut;
}