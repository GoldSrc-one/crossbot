#pragma once

#include <extdll.h>

#include <string>

namespace BrainSites
{

void Forget();

std::string Label(int site);

bool Labeled(int site);

int Find(const char *label);

std::string PlaceAt(const Vector &p);

bool FindPlace(const char *name, Vector &point, std::string &matched);

std::string PlaceNames();

}
