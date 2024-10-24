#ifndef PARAMS_H
#define PARAMS_H

#include <string>

typedef struct structParams {
    int timeLimit;
    std::string inputFile;
    bool useBoundL2;
} Params;
#endif // PARAMS_H
