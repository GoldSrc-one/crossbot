#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

#include "../game/game_profile.h"
#include "../nav/nav_mesh.h"
#include "bot_goal.h"

struct CBot;

namespace BotItems
{
	struct Placement
	{
		uint32_t areaId = 0;
		Vector point;
	};

	Vector ClampToArea(const Nav::Area &area, const Vector &point);

	const Nav::Area *StandArea(const Nav::NavMesh &mesh, const Vector &center);

	void VolumeAreas(const Nav::NavMesh &mesh, edict_t *ent, const Vector &from,
		std::vector<Placement> &out);

	bool IsPickupGone(edict_t *ent);
	bool IsChargerEmpty(edict_t *ent);
	bool IsVolumeInactive(edict_t *ent);
	bool OriginInsideVolume(edict_t *ent, const Vector &origin, int hull);
	bool IsUnavailable(edict_t *ent, ItemKind kind);

	Vector ItemCenter(edict_t *ent, ItemKind kind);

	ItemKind Classify(edict_t *ent);
	float ChargerValue(edict_t *ent, ItemKind kind, edict_t *player);
	float ChargerFull(edict_t *ent, ItemKind kind);

	void NoteRespawning(CBot &bot, edict_t *ent, ItemKind kind);
	void NoteIgnored(CBot &bot, edict_t *ent, float seconds);
	void NoteFull(CBot &bot, const char *classname);

	void RefreshMemos(CBot &bot);

	void ForgetGoalItem(CBot &bot);
	void ClearGoalItem(CBot &bot);

	bool Enabled();

	void Collect(const CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &costs,
		std::vector<BotGoal::Candidate> &out);

	void KeepPreferred(std::vector<BotGoal::Candidate> &cands);

	void Accept(CBot &bot, const BotGoal::Candidate &cand);

	bool UpdateGoalItem(CBot &bot);

	void OnTouch(CBot &bot, edict_t *ent);

	void OnArrived(CBot &bot);

	void OnRepathFailed(CBot &bot);

	void OnBlocked(CBot &bot, const char *what);

	void OnRespawn(CBot &bot);

	void PrintCandidates(const Nav::NavMesh &mesh);
}
