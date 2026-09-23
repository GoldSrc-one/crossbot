#include <extdll.h>
#include <meta_api.h>

#include <cstdio>

#include "bot.h"
#include "bot_ability.h"

namespace BotAbility
{

const char *TaskName(Task task)
{
	switch (task)
	{
	case TASK_HEAL:    return "heal";
	case TASK_REPAIR:  return "repair";
	case TASK_BUILD:   return "build";
	case TASK_STAB:    return "stab";
	case TASK_PIPES:   return "pipes";
	case TASK_PAD:     return "pad";
	case TASK_TOUCH:   return "touch";
	case TASK_DISARM:  return "disarm";
	case TASK_DETPACK: return "detpack";
	case TASK_DISCARD: return "discard";
	default:           return "none";
	}
}

void Reset(CBot &bot)
{
	State &st = bot.ability;
	State fresh;
	fresh.heals = st.heals;
	fresh.repairs = st.repairs;
	fresh.builds = st.builds;
	fresh.buildFails = st.buildFails;
	fresh.disguises = st.disguises;
	fresh.feigns = st.feigns;
	fresh.stabs = st.stabs;
	fresh.pipeBursts = st.pipeBursts;
	fresh.detonations = st.detonations;
	fresh.pads = st.pads;
	fresh.padFails = st.padFails;
	fresh.touches = st.touches;
	fresh.uncovers = st.uncovers;
	fresh.disarms = st.disarms;
	fresh.detpacks = st.detpacks;
	fresh.calls = st.calls;
	fresh.discards = st.discards;
	for (int i = 0; i < kTouchSlots; ++i)
		fresh.touchNext[i] = st.touchNext[i];
	st = fresh;
}

void PrintStatus(const CBot &bot)
{
	const State &st = bot.ability;
	char line[400];
	snprintf(line, sizeof(line),
		"CrossBot: %s ability task=%s why=%s class=%d heals=%u repairs=%u builds=%u buildfails=%u "
		"disguises=%u feigns=%u stabs=%u pipes=%u dets=%u pads=%u padfails=%u touches=%u uncovers=%u "
		"disarms=%u detpacks=%u calls=%u discards=%u\n",
		bot.name, TaskName(st.task), st.why, bot.pEdict ? bot.pEdict->v.playerclass : 0, st.heals,
		st.repairs, st.builds, st.buildFails, st.disguises, st.feigns, st.stabs, st.pipeBursts,
		st.detonations, st.pads, st.padFails, st.touches, st.uncovers, st.disarms, st.detpacks, st.calls,
		st.discards);
	SERVER_PRINT(line);
}

}
