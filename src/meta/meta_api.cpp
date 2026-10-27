#include <extdll.h>
#include <meta_api.h>

#include "../brain/brain.h"
#include "../bot/bot_sight.h"
#include "../bot/bot_stall.h"
#include "../nav/plan_worker.h"

meta_globals_t *gpMetaGlobals;
gamedll_funcs_t *gpGamedllFuncs;
mutil_funcs_t *gpMetaUtilFuncs;
enginefuncs_t *g_pengfuncsTable;

plugin_info_t Plugin_info =
{
	META_INTERFACE_VERSION,
	"CrossBot",
	"0.1",
	__DATE__,
	"crossbot",
	"https://github.com/",
	"CROSSBOT",
	PT_ANYTIME,
	PT_ANYTIME,
};

C_DLLEXPORT int Meta_Query(char *interfaceVersion, plugin_info_t **plinfo, mutil_funcs_t *pMetaUtilFuncs)
{
	*plinfo = &Plugin_info;
	gpMetaUtilFuncs = pMetaUtilFuncs;
	return TRUE;
}

META_FUNCTIONS gMetaFunctionTable =
{
	NULL,
	NULL,
	GetEntityAPI2,
	GetEntityAPI2_Post,
	GetNewDLLFunctions,
	GetNewDLLFunctions_Post,
	GetEngineFunctions,
	GetEngineFunctions_Post,
};

C_DLLEXPORT int Meta_Attach(PLUG_LOADTIME now, META_FUNCTIONS *pFunctionTable, meta_globals_t *pMGlobals, gamedll_funcs_t *pGamedllFuncs)
{
	gpMetaGlobals = pMGlobals;
	gpGamedllFuncs = pGamedllFuncs;

	g_engfuncs.pfnServerPrint("CrossBot: attached.\n");

	memcpy(pFunctionTable, &gMetaFunctionTable, sizeof(META_FUNCTIONS));
	return TRUE;
}

C_DLLEXPORT int Meta_Detach(PLUG_LOADTIME now, PL_UNLOAD_REASON reason)
{
	BotStall::Shutdown();
	Nav::StopPlanWorker();
	Brain::Shutdown();
	BotSight::Clear();
	return TRUE;
}

