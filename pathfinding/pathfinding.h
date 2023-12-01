// pathfinding.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <vector>
#include "node.h"
void displayGrid(const std::vector<float>& grid, int width, int height, const std::vector<std::pair<double, double>>& path, float windAngle, const char* name);
