#include "vrpsolution.h"

VRPSolution::VRPSolution(VRPInstance &instance) : instance(instance) {
    routes.resize(instance.k, std::vector<int>());
    cost = 0.0;
    rootBound = 0.0;
    time = 0.0;
    cvrpCuts = 0;
    nodesExplored = 0;
    rootGap = 0.0;
    rootTime = 0.0;
}

void VRPSolution::setRoutesFromSolution(
    std::vector<std::vector<int>> &edgeCount) {
    std::vector<int> currRoute;
    int i = 0;
    int next;
    bool hasNext = true;
    routes.clear();

    while (hasNext) {
        int j = 0;
        hasNext = false;

        // get next node
        for (j = 0; j < instance.n; j++) {
            if (j != i && edgeCount[i][j] > 0) {
                next = j;
                edgeCount[i][j]--;
                edgeCount[j][i]--;
                hasNext = true;
                break;
            }
        }

        if (hasNext) {
            // back to depot
            if (j == 0) {
                routes.push_back(currRoute);
                currRoute.clear();
            }
            // adding vertex to curr route
            else {
                currRoute.push_back(j);
            }

            i = j;
        }
    }
}

void VRPSolution::print() {
    std::cout << "Solution:" << std::endl;
    std::cout << "Best primal bound: " << cost << std::endl;
    std::cout << "Root bound: " << rootBound << std::endl;
    std::cout << "Root Gap: " << rootGap << std::endl;
    std::cout << "Number of nodes explored: " << nodesExplored << std::endl;
    std::cout << "Number of capacity cuts: " << cvrpCuts << std::endl;
    std::cout << "Root time: " << rootTime / 1000.0 << " s" << std::endl;
    std::cout << "Total time: " << time / 1000.0 << " s" << std::endl
              << std::endl;

    for (int i = 0; i < routes.size(); i++) {
        std::cout << "Route " << i + 1 << ": ";
        for (int id : routes[i]) {
            std::cout << std::to_string(id) << " ";
        }
        std::cout << std::endl;
    }
}