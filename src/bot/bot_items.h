#pragma once

#include <extdll.h>

#include "../game/game_profile.h"
#include "../nav/nav_mesh.h"

struct CBot;

namespace BotItems
{
	bool IsPickupGone(edict_t *ent);
	bool IsChargerEmpty(edict_t *ent);
	bool IsUnavailable(edict_t *ent, ItemKind kind);

	Vector ItemCenter(edict_t *ent, ItemKind kind);

	void NoteRespawning(CBot &bot, edict_t *ent, ItemKind kind);
	void NoteIgnored(CBot &bot, edict_t *ent, float seconds);
	void NoteFull(CBot &bot, const char *classname);

	void RefreshMemos(CBot &bot);

	void ForgetGoalItem(CBot &bot);
	void ClearGoalItem(CBot &bot);

	bool Enabled();

	bool SelectGoal(CBot &bot, const Nav::NavMesh &mesh);

	bool UpdateGoalItem(CBot &bot);

	void OnTouch(CBot &bot, edict_t *ent);

	void OnArrived(CBot &bot);

	void OnRepathFailed(CBot &bot);

	void OnRespawn(CBot &bot);

	void PrintCandidates(const Nav::NavMesh &mesh);
}
