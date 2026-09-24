#pragma once

#include "../game/weapon_spec.h"
#include "../nav/pm_sim.h"
#include "../nav/plan_search.h"
#include "bot.h"

float Bot_NormalizeAngle(float a);

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove);

void Bot_BeginFrame();

float Bot_FrameTime();

byte Bot_FrameMsec();

bool Bot_HasLongJumpModule(edict_t *pEdict);

Vector Bot_YawDir(float yawDeg);

bool Bot_TraceHitEntity(const TraceResult &tr);

PmSim::PmTraceFn Bot_EngineTraceFn(edict_t *ignore, int hull);

PmSim::PmPushFn Bot_PushFields(int hull);

Vector Bot_DriftVelocity(edict_t *pEdict, const PmSim::PmPushFn &push);

float Bot_GravityMultiplier(edict_t *pEdict);

float Bot_Carefulness(const CBot &bot);

PmSim::FallRule Bot_FallDamage(const CBot &bot);

WeaponRangeClass Bot_RangeClass(const CBot &bot);

NavPlan::RangeBias Bot_RangeBias(const CBot &bot, bool readiness = true);

void Bot_NoteMoveMaxSpeed(int playerIndex, float maxspeed);

void Bot_ForgetMoveMaxSpeed(edict_t *pEdict);

bool Bot_MoveMaxSpeedKnown(edict_t *pEdict);

bool Bot_Frozen(const edict_t *pEdict);

float Bot_MaxSpeed(edict_t *pEdict);

PmSim::PmPhysics Bot_Physics(edict_t *pEdict);

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump = false, float upmove = 0.f);
