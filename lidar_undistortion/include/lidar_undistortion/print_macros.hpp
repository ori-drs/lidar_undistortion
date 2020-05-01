#pragma once

#if DEBUG_MODE
#define DEBUG_PRINTLN(x) std::cerr << x << std::endl
#else
#define DEBUG_PRINTLN(x) void (0)
#endif

