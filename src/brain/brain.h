#pragma once

#include <string>

namespace Brain
{

void RegisterCvars();

void OnMapStart();

void OnMapEnd();

void Frame();

void Shutdown();

void PrintStatus();

void Reload();

void Unignore(const char *name);

void IgnorePlayer(const char *name, float seconds);

bool Hears(int userid);

std::string TraceDir();

}
