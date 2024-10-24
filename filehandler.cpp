#include "filehandler.h"

namespace filehandler {
void readInstance(Params &params, VRPInstance &instance) {
    std::ifstream file;
    std::string line;
    std::vector<std::string> lines;
    std::vector<double> demands;
    bool hasDistanceMatrixSection = false;

    file.open(params.inputFile.c_str());
    if (!file) {
        std::cerr << "File '" << params.inputFile << "' does not exist.\n";
        exit(0);
    }

    // put file lines in an array
    while (std::getline(file, line)) {
        lines.push_back(line);
    }

    // read each line
    int i = 0;
    while (i < lines.size()) {
        line = lines[i];

        if (line.find("NAME") != std::string::npos) {
            std::smatch match;
            std::regex_search(line, match, std::regex("n([0-9]+)-k([0-9]+)"));

            if (match.str(2) != "") {
                instance.k = stoi(match.str(2));
            }
        } else if (line.find("DIMENSION") != std::string::npos) {
            std::smatch match;
            std::regex_search(line, match, std::regex("[0-9]+"));
            instance.n = stoi(match.str(0));
            instance.m = (instance.n * (instance.n - 1)) / 2;
            instance.distances.resize(instance.n, std::vector<int>(instance.n));
            instance.posx.resize(instance.n);
            instance.posy.resize(instance.n);
            instance.demands.resize(instance.n);
        } else if (line.find("CAPACITY") != std::string::npos) {
            std::smatch match;
            std::regex_search(line, match, std::regex("[0-9]+"));
            instance.capacity = stoi(match.str(0));
        } else if (line.find("N_VEHICLES") != std::string::npos) {
            std::smatch match;
            std::regex_search(line, match, std::regex("[0-9]+"));
            instance.k = stoi(match.str(0));
        } else if (line.find("NODE_COORD_SECTION") != std::string::npos) {
            std::smatch match;
            int id;
            double posx, posy;

            line = lines[++i];
            while (std::regex_search(
                line, match,
                std::regex("\\s*([-+]?[0-9]+)\\s*([-+]?[0-9]+)"
                           "\\s*([-+]?[0-9]+)\\s*"))) {
                id = std::stoi(match.str(1));
                posx = std::stof(match.str(2));
                posy = std::stof(match.str(3));

                instance.posx[id - 1] = posx;
                instance.posy[id - 1] = posy;

                line = lines[++i];
            }
            i--;
        } else if (line.find("DEMAND_SECTION") != std::string::npos) {
            std::smatch match;
            int id;
            double demand;

            line = lines[++i];
            while (std::regex_search(
                line, match,
                std::regex("\\s*([0-9]+)\\s*([0-9]+\\.?[0-9]*)\\s*"))) {
                id = std::stoi(match.str(1));
                demand = std::stof(match.str(2));
                instance.demands[id - 1] = demand;

                line = lines[++i];
            }
            i--;
        } else if (line.find("DISTANCE_MATRIX") != std::string::npos) {
            hasDistanceMatrixSection = true;
            line = lines[++i];
            double roundFactor = 1e6;

            for (int j = 0; j < instance.n; j++) {
                std::stringstream ss(line);
                std::string s;
                int k = 0;

                while (std::getline(ss, s, ' ')) {
                    if (k < instance.n) {
                        instance.distances[j][k] =
                            round(std::stof(s) * roundFactor) / roundFactor;
                    }
                    k++;
                }

                line = lines[++i];
            }
            i--;
        }

        // go to next line
        i++;
    }

    // If needed, compute edge costs from positions.
    if (!hasDistanceMatrixSection) {
        for (int i = 0; i < instance.n; i++) {
            for (int j = 0; j < instance.n; j++) {
                if (j == i) {
                    continue;
                }

                instance.distances[i][j] =
                    std::round(hypot(abs(instance.posx[i] - instance.posx[j]),
                                     abs(instance.posy[i] - instance.posy[j])));
            }
        }
    }
}
} // namespace filehandler
