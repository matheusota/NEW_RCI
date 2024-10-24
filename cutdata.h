#ifndef CUTDATA_H
#define CUTDATA_H

#include <vector>
#include <utility>
#include <string>

struct CutData {
    std::vector<std::tuple<int, int, double>> coefs;
    double RHS;
    double LHS;
    char sense;
    std::string name;
};

#endif // CUTDATA_H