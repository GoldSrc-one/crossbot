#pragma once

#include "bot.h"

#include "../nav/airmove_cache.h"
#include "../nav/pm_sim.h"

void Bot_MoveNav(CBot &bot);

void Bot_ResetNavPath(CBot &bot);

bool Bot_ManeuverActive(const CBot &bot);

void Bot_ResetAirmoveCache();

const NavPlan::AirmoveCache &Bot_AirmoveCache();

PmSim::PmPhysics Bot_DrivePhysics(edict_t *pEdict);

bool Bot_PenalizeCurrentConnection(CBot &bot);

