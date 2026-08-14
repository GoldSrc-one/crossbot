#include <extdll.h>
#include <meta_api.h>

#include "../bot/bot_manager.h"
#include "../console/commands.h"

cvar_t g_tracePathCvar = {
	"crossbot_trace_path",
	"valve/crossbot_data/nav_trace.txt",
	FCVAR_SERVER
};

cvar_t g_navAvoidCvar = {
	"crossbot_nav_avoid",
	"1",
	FCVAR_SERVER
};

cvar_t g_gatherCvar = {
	"crossbot_gather",
	"1",
	FCVAR_SERVER
};

cvar_t g_gatherDebugCvar = {
	"crossbot_gather_debug",
	"0",
	FCVAR_SERVER
};

void GameDLLInit()
{
	CrossBotCommands::Register();
	g_engfuncs.pfnCvar_RegisterVariable(&g_tracePathCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_navAvoidCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_gatherDebugCvar);
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

void Touch(edict_t *pentTouched, edict_t *pentOther)
{
	BotManager::Touch(pentTouched, pentOther);
	RETURN_META(MRES_IGNORED);
}

DLL_FUNCTIONS g_DllFunctionTable =
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

DLL_FUNCTIONS g_DllFunctionTable_Post =
{
	GameDLLInit,
	NULL,
	NULL,
	NULL,
	Touch,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	ClientDisconnect,
	NULL,
	ClientPutInServer,
	NULL,
	NULL,
	ServerActivate,
	ServerDeactivate,
	NULL,
	NULL,
	StartFrame,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

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

	memcpy(pFunctionTable, &g_DllFunctionTable, sizeof(DLL_FUNCTIONS));
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

	memcpy(pFunctionTable, &g_DllFunctionTable_Post, sizeof(DLL_FUNCTIONS));
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

