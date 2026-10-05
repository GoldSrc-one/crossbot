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

void Bot_BailManeuver(CBot &bot, const char *why);

bool Bot_SimulateAirmoveArc(edict_t *pEdict, const Vector &origin, const Vector *velocity, float yaw,
	float speed, bool jump, int hull, float frameTime, float maxSeconds, BotAirmoveArc &out);

bool Bot_AirmoveArcAt(const BotAirmoveArc &arc, float t, Vector &pos, Vector &vel);

void Bot_FaceDir(CBot &bot, const Vector &dir);

void Bot_DriveTo(CBot &bot, const Vector &angles, const Vector &to, bool stop,
	unsigned short extra, float speed, float standRadius);

