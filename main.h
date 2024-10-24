#ifndef MAIN_H
#define MAIN_H

#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "filehandler.h"
#include "params.h"
#include "vrpinstance.h"
#include "vrpsolution.h"
#include "mygrbmodel.h"

void readParams(Params &params, int argc, char *argv[]);

#endif
