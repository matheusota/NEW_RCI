#ifndef FILEHANDLER_H
#define FILEHANDLER_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <regex>
#include <algorithm>
#include <cmath>
#include "params.h"
#include "vrpinstance.h"

namespace filehandler {
void readInstance(Params &params, VRPInstance &instance);
} // namespace filehandler

#endif // FILEHANDLER_H
