#pragma once

namespace PlayerModels
{


bool PickRandom(int iGame, char *out, unsigned int size);

bool Installed(int iGame, const char *name);

void PrintList(int iGame);

void Forget();

}
