#pragma once

#include "bot.h"

// Drives one bot's movement for the current frame via pfnRunPlayerMove:
// a wall-avoiding random walk. Not real navigation -- placeholder plumbing
// proving the CREATE_FAKE_CLIENT / pfnRunPlayerMove pipeline works end to end.
void Bot_Move(CBot &bot);
