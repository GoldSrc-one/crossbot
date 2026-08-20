#pragma once

#include <extdll.h>

namespace FakeCmd
{

constexpr int kMaxArgs = 8;

constexpr int kMaxLength = 128;

bool Run(edict_t *player, const char *cmd);

const char *Args();

const char *Argv(int argc);

int Argc();

}
