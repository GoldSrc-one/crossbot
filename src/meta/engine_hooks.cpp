#include <extdll.h>
#include <meta_api.h>

#include "../bot/bot_manager.h"

void EmitSound(edict_t *entity, int channel, const char *sample, float volume, float attenuation,
	int fFlags, int pitch)
{
	BotManager::OnSound(entity, channel, sample, volume, attenuation);
	RETURN_META(MRES_IGNORED);
}

void PlaybackEvent(int flags, const edict_t *pInvoker, unsigned short eventindex, float delay,
	float *origin, float *angles, float fparam1, float fparam2, int iparam1, int iparam2,
	int bparam1, int bparam2)
{
	BotManager::OnEvent(flags, pInvoker);
	RETURN_META(MRES_IGNORED);
}

C_DLLEXPORT int GetEngineFunctions(enginefuncs_t *pengfuncsFromEngine, int *interfaceVersion)
{
	if (!pengfuncsFromEngine) {
		ALERT(at_logged, "%s called with null pengfuncsFromEngine", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != ENGINE_INTERFACE_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, ENGINE_INTERFACE_VERSION);
		*interfaceVersion = ENGINE_INTERFACE_VERSION;
		return FALSE;
	}

	memset(pengfuncsFromEngine, 0, sizeof(enginefuncs_t));
	return TRUE;
}

C_DLLEXPORT int GetEngineFunctions_Post(enginefuncs_t *pengfuncsFromEngine, int *interfaceVersion)
{
	if (!pengfuncsFromEngine) {
		ALERT(at_logged, "%s called with null pengfuncsFromEngine", __FUNCTION__);
		return FALSE;
	}
	if (*interfaceVersion != ENGINE_INTERFACE_VERSION) {
		ALERT(at_logged, "%s version mismatch; requested=%d ours=%d", __FUNCTION__, *interfaceVersion, ENGINE_INTERFACE_VERSION);
		*interfaceVersion = ENGINE_INTERFACE_VERSION;
		return FALSE;
	}

	memset(pengfuncsFromEngine, 0, sizeof(enginefuncs_t));
	pengfuncsFromEngine->pfnEmitSound = EmitSound;
	pengfuncsFromEngine->pfnPlaybackEvent = PlaybackEvent;
	return TRUE;
}
