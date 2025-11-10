#pragma once
#include <iostream>
#define LOG_I(msg) std::cout << "[I] " << msg << std::endl
#define LOG_W(msg) std::cout << "[W] " << msg << std::endl
#define LOG_E(msg) std::cerr << "[E] " << msg << std::endl
