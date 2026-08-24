#pragma once

#include "bot.h"

#include "../nav/pm_sim.h"

void Bot_MoveNav(CBot &bot);

void Bot_ResetNavPath(CBot &bot);

PmSim::PmPhysics Bot_DrivePhysics(edict_t *pEdict);

bool Bot_PenalizeCurrentConnection(CBot &bot);

