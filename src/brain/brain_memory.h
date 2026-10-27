#pragma once

#include <string>
#include <vector>

namespace BrainMemory
{

void Add(const char *text, const char *players, const char *map, int round);

std::vector<std::string> Select(const char *speaker, const char *map, int max);

int Count();

void Print(int count);

}
