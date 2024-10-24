#include "main.h"

int main(int argc, char *argv[]) {
    // Set parameters from stdin.
    Params params;
    readParams(params, argc, argv);

    // Read the problem instance from input file.
    VRPInstance instance;
    filehandler::readInstance(params, instance);

    // Initialize a solution.
    VRPSolution solution(instance);

    // Run the algorithm.
    auto started = std::chrono::high_resolution_clock::now();
    MyGRBModel gurobiSolver(instance, params);
    gurobiSolver.solve(solution);
    auto done = std::chrono::high_resolution_clock::now();
    double time = double(std::chrono::duration_cast<std::chrono::milliseconds>(
                             done - started)
                             .count()) /
                  1000.0;

    // Print solution.
    solution.print();

    return 0;
}

void readParams(Params &params, int argc, char *argv[]) {
    params.timeLimit = 1800;
    params.useBoundL2 = false;

    // Read
    for (int i = 1; i < argc; i++) {
        const std::string arg(argv[i]);
        std::string next;
        if ((i + 1) < argc) {
            next = std::string(argv[i + 1]);
        } else {
            next = std::string("");
        }

        if (arg.find("-l2") == 0 && next.size() > 0) {
            params.useBoundL2 = true;
            continue;
        }

        else if (arg.find("-t") == 0 && next.size() > 0) {
            params.timeLimit = atoi(next.c_str());
            i++;
            continue;
        }

        else if (arg.find("-i") == 0 && next.size() > 0) {
            params.inputFile = next;
            i++;
            continue;
        }

        std::cout << "Invalid parameter: \"" << arg
                  << "\" (or missing parameters)" << std::endl;
    }

    // Check
    if (params.inputFile.size() < 1) {
        std::cout << "Invalid input file name" << std::endl;
    }
}
