#ifndef VRPINSTANCE_H
#define VRPINSTANCE_H

#include <memory>
#include <vector>

class VRPInstance {
  public:
    int n;
    int m;
    int k;
    int capacity;
    std::vector<std::vector<int>> distances;
    std::vector<double> posx;
    std::vector<double> posy;
    std::vector<int> demands;
};
#endif // VRPINSTANCE_H
