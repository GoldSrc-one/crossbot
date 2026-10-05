#pragma once

#include <extdll.h>

#include <vector>

struct CBot;

namespace Nav
{
struct NavMesh;
}

namespace TFCAbility
{

bool Run(CBot &bot, const Nav::NavMesh &mesh);

void RefreshTeleporters();

bool FeignHold(edict_t *pEdict);

bool Disguised(edict_t *ent);

bool HoldFire(const CBot &bot);

int ClassifyItem(edict_t *ent);

bool ItemUsable(edict_t *player, edict_t *item);

float ItemWorth(edict_t *player, edict_t *item, float ammoNeed);

float PackWait(edict_t *item);

float StationValue(edict_t *player);

bool Fortifies(edict_t *ent);

bool CanDetpack(edict_t *ent);

void NoteSound(edict_t *src, const char *sample);

void Patients(edict_t *medic, std::vector<edict_t *> &out);

}
