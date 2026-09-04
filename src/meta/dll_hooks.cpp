#include <extdll.h>
#include <meta_api.h>
#include <pm_defs.h>

#include "../bot/bot_manager.h"
#include "../bot/bot_perf.h"
#include "../bot/bot_sense.h"
#include "../bot/bot_timing.h"
#include "../console/commands.h"
#include "../game/gamedata.h"

cvar_t g_tracePathCvar = {
	"crossbot_trace_path",
	"valve/addons/crossbot/trace/nav_trace.txt",
	FCVAR_SERVER
};

cvar_t g_navAvoidCvar = {
	"crossbot_nav_avoid",
	"1",
	FCVAR_SERVER
};

cvar_t g_navBreakCvar = {
	"crossbot_nav_break",
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

cvar_t g_combatDebugCvar = {
	"crossbot_combat_debug",
	"0",
	FCVAR_SERVER
};

cvar_t g_ambushCvar = {
	"crossbot_ambush",
	"1",
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

cvar_t g_goalBiasCvar = {
	"crossbot_goal_bias",
	"2",
	FCVAR_SERVER
};

cvar_t g_detourBudgetCvar = {
	"crossbot_detour_budget",
	"6",
	FCVAR_SERVER
};

cvar_t g_oppoJumpCvar = {
	"crossbot_oppo_jump",
	"1",
	FCVAR_SERVER
};

cvar_t g_aimDelayCvar = {
	"crossbot_aim_delay",
	"0.3",
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

cvar_t g_aimErrorCvar = {
	"crossbot_aim_error",
	"1",
	FCVAR_SERVER
};

cvar_t g_fireCvar = {
	"crossbot_fire",
	"1",
	FCVAR_SERVER
};

cvar_t g_aimDriftCvar = {
	"crossbot_aim_drift",
	"1",
	FCVAR_SERVER
};

cvar_t g_aimQuantumCvar = {
	"crossbot_aim_quantum",
	"0.4476",
	FCVAR_SERVER
};

cvar_t g_aimImpulseCvar = {
	"crossbot_aim_impulse",
	"15",
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

cvar_t g_carefulPinCvar = {
	"crossbot_careful_pin",
	"-1",
	FCVAR_SERVER
};

cvar_t g_deployCvar = {
	"crossbot_deploy",
	"1",
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

cvar_t g_airmoveCorrectCvar = {
	"crossbot_airmove_correct",
	"1",
	FCVAR_SERVER
};

#ifdef CROSSBOT_BSPTRACE

cvar_t g_bspTraceCvar = {
	"crossbot_bsptrace",
	"0",
	FCVAR_SERVER
};

cvar_t g_bspTraceBlockersCvar = {
	"crossbot_bsptrace_blockers",
	"0",
	FCVAR_SERVER
};

#endif

cvar_t g_timeDebugCvar = {
	"crossbot_time_debug",
	"0",
	FCVAR_SERVER
};

#ifdef CROSSBOT_VERBOSE

cvar_t g_verboseCvar = {
	"crossbot_verbose",
	"1",
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
	GameData::Load();
	CrossBotCommands::Register();
	g_engfuncs.pfnCvar_RegisterVariable(&g_tracePathCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_navAvoidCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_navBreakCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_combatCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_combatDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_ambushCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_funCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_funDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_goalBiasCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_detourBudgetCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_oppoJumpCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimDelayCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_senseCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_senseDecayCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimErrorCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimDriftCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimQuantumCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_aimImpulseCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_fireCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_vantageCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_carefulCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_carefulPinCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deployCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deploySurfaceCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_deployDebugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_timeDebugCvar);
#ifdef CROSSBOT_BSPTRACE
	g_engfuncs.pfnCvar_RegisterVariable(&g_bspTraceCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_bspTraceBlockersCvar);
#endif
	g_engfuncs.pfnCvar_RegisterVariable(&g_airmoveVerifyCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_airmoveCorrectCvar);
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
	BotManager::ServerActivate(pEdictList, edictCount, clientMax);
	RETURN_META(MRES_IGNORED);
}

void ServerDeactivate()
{
	BotManager::ServerDeactivate();
	RETURN_META(MRES_IGNORED);
}

void ClientPutInServer(edict_t *pEntity)
{
	BotManager::ClientPutInServer(pEntity);
	RETURN_META(MRES_IGNORED);
}

void ClientDisconnect(edict_t *pEntity)
{
	BotManager::ClientDisconnect(pEntity);
	RETURN_META(MRES_IGNORED);
}

void StartFrame()
{
	BotManager::StartFrame();
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
	if (ppmove && ppmove->PM_PlaySound == PM_PlaySoundHook)
		ppmove->PM_PlaySound = g_pmPlaySound;
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

