#include <extdll.h>
#include <meta_api.h>
#include <pm_defs.h>

#include "../bot/bot_hitch.h"
#include "../bot/bot_manager.h"
#include "../bot/bot_move.h"
#include "../bot/bot_perf.h"
#include "../bot/bot_sense.h"
#include "../bot/bot_timing.h"
#include "../brain/brain.h"
#include "../brain/brain_feed.h"
#include "../console/commands.h"
#include "../game/gamedata.h"

cvar_t g_tracePathCvar = {
	"crossbot_trace_path",
	"",
	FCVAR_SERVER
};

cvar_t g_navAvoidCvar = {
	"crossbot_nav_avoid",
	"1",
	FCVAR_SERVER
};

cvar_t g_beamJumpCvar = {
	"crossbot_beam_jump",
	"1",
	FCVAR_SERVER
};

cvar_t g_gatherCvar = {
	"crossbot_gather",
	"1",
	FCVAR_SERVER
};

cvar_t g_combatCvar = {
	"crossbot_combat",
	"1",
	FCVAR_SERVER
};

cvar_t g_teamsCvar = {
	"crossbot_teams",
	"1",
	FCVAR_SERVER
};

cvar_t g_combatDebugCvar = {
	"crossbot_combat_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_ambushCvar = {
	"crossbot_ambush",
	"-1",
	FCVAR_SERVER
};

cvar_t g_gatherDebugCvar = {
	"crossbot_gather_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_funCvar = {
	"crossbot_fun",
	"1",
	FCVAR_SERVER
};

cvar_t g_funDebugCvar = {
	"crossbot_fun_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_strategicCvar = {
	"crossbot_strategic",
	"1",
	FCVAR_SERVER
};

cvar_t g_strategicDebugCvar = {
	"crossbot_strategic_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_objectivesCvar = {
	"crossbot_objectives",
	"1",
	FCVAR_SERVER
};

cvar_t g_tacticsCvar = {
	"crossbot_tactics",
	"1",
	FCVAR_SERVER
};

cvar_t g_tacticsDebugCvar = {
	"crossbot_tactics_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_snipeCvar = {
	"crossbot_snipe",
	"1",
	FCVAR_SERVER
};

cvar_t g_campCvar = {
	"crossbot_camp",
	"-1",
	FCVAR_SERVER
};

cvar_t g_campDebugCvar = {
	"crossbot_camp_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_difficultyCvar = {
	"crossbot_difficulty",
	"1",
	FCVAR_SERVER
};

cvar_t g_oppoIntervalCvar = {
	"crossbot_oppo_interval",
	"-1",
	FCVAR_SERVER
};

cvar_t g_turnScaleCvar = {
	"crossbot_turn_scale",
	"-1",
	FCVAR_SERVER
};

cvar_t g_aimGainCvar = {
	"crossbot_aim_gain",
	"-1",
	FCVAR_SERVER
};

cvar_t g_backCheckCvar = {
	"crossbot_back_check",
	"-1",
	FCVAR_SERVER
};

cvar_t g_headPctCvar = {
	"crossbot_head_pct",
	"-1",
	FCVAR_SERVER
};

cvar_t g_predictPctCvar = {
	"crossbot_predict_pct",
	"-1",
	FCVAR_SERVER
};

cvar_t g_combatInterruptCvar = {
	"crossbot_combat_interrupt",
	"-1",
	FCVAR_SERVER
};

cvar_t g_pickupPriorityCvar = {
	"crossbot_pickup_priority",
	"-1",
	FCVAR_SERVER
};

cvar_t g_gaussCvar = {
	"crossbot_gauss",
	"-1",
	FCVAR_SERVER
};

cvar_t g_bunnyHopCvar = {
	"crossbot_bunnyhop",
	"-1",
	FCVAR_SERVER
};

cvar_t g_grenadeCookCvar = {
	"crossbot_grenade_cook",
	"-1",
	FCVAR_SERVER
};

cvar_t g_backstabCvar = {
	"crossbot_backstab",
	"-1",
	FCVAR_SERVER
};

cvar_t g_braceMovingCvar = {
	"crossbot_brace_moving",
	"-1",
	FCVAR_SERVER
};

cvar_t g_wallbangCvar = {
	"crossbot_wallbang",
	"-1",
	FCVAR_SERVER
};

cvar_t g_spreadFitCvar = {
	"crossbot_spread_fit",
	"-1",
	FCVAR_SERVER
};

cvar_t g_aimDelayCvar = {
	"crossbot_aim_delay",
	"-1",
	FCVAR_SERVER
};

cvar_t g_noticeCvar = {
	"crossbot_notice",
	"-1",
	FCVAR_SERVER
};

cvar_t g_senseCvar = {
	"crossbot_sense",
	"1",
	FCVAR_SERVER
};

cvar_t g_senseDecayCvar = {
	"crossbot_sense_decay",
	"5",
	FCVAR_SERVER
};

cvar_t g_coverageCvar = {
	"crossbot_coverage",
	"1",
	FCVAR_SERVER
};

cvar_t g_fireCvar = {
	"crossbot_fire",
	"1",
	FCVAR_SERVER
};

cvar_t g_vantageCvar = {
	"crossbot_vantage",
	"1",
	FCVAR_SERVER
};

cvar_t g_carefulCvar = {
	"crossbot_careful",
	"1",
	FCVAR_SERVER
};

cvar_t g_revengeCvar = {
	"crossbot_revenge",
	"1",
	FCVAR_SERVER
};

cvar_t g_revengeBudgetCvar = {
	"crossbot_revenge_budget",
	"10",
	FCVAR_SERVER
};

cvar_t g_revengeFovCvar = {
	"crossbot_revenge_fov",
	"120",
	FCVAR_SERVER
};

cvar_t g_revengeLosRadiusCvar = {
	"crossbot_revenge_los_radius",
	"2048",
	FCVAR_SERVER
};

cvar_t g_revengeDebugCvar = {
	"crossbot_revenge_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_grenadeCvar = {
	"crossbot_grenade",
	"1",
	FCVAR_SERVER
};

cvar_t g_deployCvar = {
	"crossbot_deploy",
	"1",
	FCVAR_SERVER
};

cvar_t g_meleeOnlyCvar = {
	"crossbot_melee_only",
	"0",
	FCVAR_SERVER
};

cvar_t g_namePrefixCvar = {
	"crossbot_name_prefix",
	"",
	FCVAR_SERVER
};

cvar_t g_deploySurfaceCvar = {
	"crossbot_deploy_surface",
	"0",
	FCVAR_SERVER
};

cvar_t g_deployDebugCvar = {
	"crossbot_deploy_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_airmoveVerifyCvar = {
	"crossbot_airmove_verify",
	"0",
	FCVAR_SERVER
};

cvar_t g_timeDebugCvar = {
	"crossbot_time_debug",
	"0",
	FCVAR_SERVER
};

#ifdef CROSSBOT_VERBOSE

cvar_t g_verboseCvar = {
	"crossbot_verbose",
	"0",
	FCVAR_SERVER
};

#endif

#ifdef CROSSBOT_PERF

cvar_t g_perfCvar = {
	"crossbot_perf",
	"0",
	FCVAR_SERVER
};

cvar_t g_perfSpikeCvar = {
	"crossbot_perf_spike",
	"5",
	FCVAR_SERVER
};

#endif

void GameDLLInit()
{
	BotPerf::InstallTraceWrappers();
	GameData::LoadAll();
	CrossBotCommands::Register();
	g_engfuncs.pfnCvar_RegisterVariable(&g_tracePathCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_navAvoidCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_beamJumpCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_combatCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_teamsCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_combatDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_ambushCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_funCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_funDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_strategicCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_strategicDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_objectivesCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_tacticsCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_tacticsDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_snipeCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_campCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_campDebugCvar);	g_engfuncs.pfnCvar_RegisterVariable(&g_difficultyCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_oppoIntervalCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_turnScaleCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimGainCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_backCheckCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_headPctCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_predictPctCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_combatInterruptCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_pickupPriorityCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gaussCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_bunnyHopCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_grenadeCookCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_backstabCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_braceMovingCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_wallbangCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_spreadFitCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimDelayCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_noticeCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_senseCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_senseDecayCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_coverageCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_fireCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_vantageCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_carefulCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_revengeCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_revengeBudgetCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_revengeFovCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_revengeLosRadiusCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_revengeDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deployCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_meleeOnlyCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_namePrefixCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_grenadeCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deploySurfaceCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deployDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_timeDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_airmoveVerifyCvar);
	Brain::RegisterCvars();
	BotHitch::Register();
#ifdef CROSSBOT_VERBOSE
	g_engfuncs.pfnCvar_RegisterVariable(&g_verboseCvar);
#endif
#ifdef CROSSBOT_PERF
	g_engfuncs.pfnCvar_RegisterVariable(&g_perfCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_perfSpikeCvar);
#endif
	RETURN_META(MRES_IGNORED);
}

void ServerActivate(edict_t *pEdictList, int edictCount, int clientMax)
{
	BotHitch::Reset();
	BotManager::ServerActivate(pEdictList, edictCount, clientMax);
	Brain::OnMapStart();
	RETURN_META(MRES_IGNORED);
}

void ServerDeactivate()
{
	Brain::OnMapEnd();
	BotManager::ServerDeactivate();
	BrainFeed::Reset();
	RETURN_META(MRES_IGNORED);
}

void ClientPutInServer(edict_t *pEntity)
{
	BotManager::ClientPutInServer(pEntity);
	RETURN_META(MRES_IGNORED);
}

void ClientDisconnect(edict_t *pEntity)
{
	Bot_ForgetMoveMaxSpeed(pEntity);
	BotManager::ClientDisconnect(pEntity);
	RETURN_META(MRES_IGNORED);
}

void StartFrame()
{
	BotHitch::FrameBegin();
	BotManager::StartFrame();
	BotHitch::Section(BotHitch::SECTION_BRAIN);
	Brain::Frame();
	BotHitch::FrameEnd();
	RETURN_META(MRES_IGNORED);
}

void PlayerPostThink(edict_t *pEntity)
{
	BotTiming::NotePostThink(pEntity);
	RETURN_META(MRES_IGNORED);
}

void Touch(edict_t *pentTouched, edict_t *pentOther)
{
	BotManager::Touch(pentTouched, pentOther);
	RETURN_META(MRES_IGNORED);
}

void ClientCommand(edict_t *pEntity)
{
	BrainFeed::OnClientCommand(pEntity);
	RETURN_META(MRES_IGNORED);
}

static int g_pmPlayerEnt = 0;
static void (*g_pmPlaySound)(int, const char *, float, float, int, int) = NULL;

static void PM_PlaySoundHook(int channel, const char *sample, float volume, float attenuation,
	int fFlags, int pitch)
{
	if (g_pmPlaySound)
		g_pmPlaySound(channel, sample, volume, attenuation, fFlags, pitch);

	if (g_pmPlayerEnt <= 0)
		return;

	edict_t *player = g_engfuncs.pfnPEntityOfEntIndex(g_pmPlayerEnt);
	if (player)
		BotManager::OnSound(player, channel, sample, volume, attenuation);
}

static void PM_Move(playermove_t *ppmove, qboolean server)
{
	if (ppmove && BotSense::Enabled() && ppmove->PM_PlaySound != PM_PlaySoundHook)
	{
		g_pmPlayerEnt = ppmove->player_index + 1;
		g_pmPlaySound = ppmove->PM_PlaySound;
		ppmove->PM_PlaySound = PM_PlaySoundHook;
	}
	RETURN_META(MRES_IGNORED);
}

static void PM_Move_Post(playermove_t *ppmove, qboolean server)
{
	if (ppmove)
	{
		if (ppmove->PM_PlaySound == PM_PlaySoundHook)
			ppmove->PM_PlaySound = g_pmPlaySound;
		Bot_NoteMoveMaxSpeed(ppmove->player_index, ppmove->maxspeed);
	}
	g_pmPlayerEnt = 0;
	RETURN_META(MRES_IGNORED);
}

NEW_DLL_FUNCTIONS g_NewDllFunctionTable =
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

NEW_DLL_FUNCTIONS g_NewDllFunctionTable_Post =
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

C_DLLEXPORT int GetEntityAPI2(DLL_FUNCTIONS *pFunctionTable, int *interfaceVersion)
{
	if (!pFunctionTable) {
		ALERT(at_logged, "%s called with null pFunctionTable", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != INTERFACE_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, INTERFACE_VERSION);
		*interfaceVersion = INTERFACE_VERSION;
		return FALSE;
	}

	memset(pFunctionTable, 0, sizeof(DLL_FUNCTIONS));
	pFunctionTable->pfnPM_Move = PM_Move;
	pFunctionTable->pfnClientCommand = ClientCommand;
	return TRUE;
}

C_DLLEXPORT int GetEntityAPI2_Post(DLL_FUNCTIONS *pFunctionTable, int *interfaceVersion)
{
	if (!pFunctionTable) {
		ALERT(at_logged, "%s called with null pFunctionTable", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != INTERFACE_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, INTERFACE_VERSION);
		*interfaceVersion = INTERFACE_VERSION;
		return FALSE;
	}

	memset(pFunctionTable, 0, sizeof(DLL_FUNCTIONS));
	pFunctionTable->pfnGameInit = GameDLLInit;
	pFunctionTable->pfnTouch = Touch;
	pFunctionTable->pfnClientDisconnect = ClientDisconnect;
	pFunctionTable->pfnClientPutInServer = ClientPutInServer;
	pFunctionTable->pfnServerActivate = ServerActivate;
	pFunctionTable->pfnServerDeactivate = ServerDeactivate;
	pFunctionTable->pfnStartFrame = StartFrame;
	pFunctionTable->pfnPlayerPostThink = PlayerPostThink;
	pFunctionTable->pfnPM_Move = PM_Move_Post;
	return TRUE;
}

C_DLLEXPORT int GetNewDLLFunctions(NEW_DLL_FUNCTIONS *pNewFunctionTable, int *interfaceVersion)
{
	if (!pNewFunctionTable) {
		ALERT(at_logged, "%s called with null pNewFunctionTable", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != NEW_DLL_FUNCTIONS_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, NEW_DLL_FUNCTIONS_VERSION);
		*interfaceVersion = NEW_DLL_FUNCTIONS_VERSION;
		return FALSE;
	}

	memcpy(pNewFunctionTable, &g_NewDllFunctionTable, sizeof(NEW_DLL_FUNCTIONS));
	return TRUE;
}

C_DLLEXPORT int GetNewDLLFunctions_Post(NEW_DLL_FUNCTIONS *pNewFunctionTable, int *interfaceVersion)
{
	if (!pNewFunctionTable) {
		ALERT(at_logged, "%s called with null pNewFunctionTable", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != NEW_DLL_FUNCTIONS_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, NEW_DLL_FUNCTIONS_VERSION);
		*interfaceVersion = NEW_DLL_FUNCTIONS_VERSION;
		return FALSE;
	}

	memcpy(pNewFunctionTable, &g_NewDllFunctionTable_Post, sizeof(NEW_DLL_FUNCTIONS));
	return TRUE;
}

