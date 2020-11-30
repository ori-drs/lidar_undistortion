#pragma once
#include <iostream>

#define DEBUG_MODE 0
#if DEBUG_MODE
#define DEBUG_PRINTLN(x) std::cerr << x << std::endl
#else
#define DEBUG_PRINTLN(x) void (0)
#endif

#define ERROR_PRINTLN(x) std::cerr << "ERROR: " << x << std::endl
#define INFO_PRINTLN(x) std::cout << "INFO: " << x << std::endl
