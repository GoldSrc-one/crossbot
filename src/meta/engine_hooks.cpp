#include <extdll.h>
#include <meta_api.h>

#include <cstdarg>
#include <cstdio>

#include "../bot/bot_daze.h"
#include "../bot/bot_manager.h"
#include "../bot/bot_perf.h"
#include "../bot/bot_sense.h"
#include "../brain/brain_feed.h"
#include "../game/fake_cmd.h"

#ifdef CROSSBOT_PERF

void PerfTraceLine(const float *v1, const float *v2, int fNoMonsters, edict_t *pentToSkip,
	TraceResult *ptr)
{
	BotPerf::TraceBegin(BotPerf::TRACE_LINE_T, v1, v2, fNoMonsters);
	RETURN_META(MRES_IGNORED);
}

void PerfTraceLine_Post(const float *v1, const float *v2, int fNoMonsters, edict_t *pentToSkip,
	TraceResult *ptr)
{
	BotPerf::TraceEnd();
	RETURN_META(MRES_IGNORED);
}

void PerfTraceHull(const float *v1, const float *v2, int fNoMonsters, int hullNumber,
	edict_t *pentToSkip, TraceResult *ptr)
{
	BotPerf::TraceBegin(BotPerf::TRACE_HULL_T, v1, v2, fNoMonsters);
	RETURN_META(MRES_IGNORED);
}

void PerfTraceHull_Post(const float *v1, const float *v2, int fNoMonsters, int hullNumber,
	edict_t *pentToSkip, TraceResult *ptr)
{
	BotPerf::TraceEnd();
	RETURN_META(MRES_IGNORED);
}

int PerfTraceMonsterHull(edict_t *pEdict, const float *v1, const float *v2, int fNoMonsters,
	edict_t *pentToSkip, TraceResult *ptr)
{
	BotPerf::TraceBegin(BotPerf::TRACE_MONSTER_T, v1, v2, fNoMonsters);
	RETURN_META_VALUE(MRES_IGNORED, 0);
}

int PerfTraceMonsterHull_Post(edict_t *pEdict, const float *v1, const float *v2, int fNoMonsters,
	edict_t *pentToSkip, TraceResult *ptr)
{
	BotPerf::TraceEnd();
	RETURN_META_VALUE(MRES_IGNORED, 0);
}

#endif

void EmitSound(edict_t *entity, int channel, const char *sample, float volume, float attenuation,
	int fFlags, int pitch)
{
	BotManager::OnSound(entity, channel, sample, volume, attenuation);
	RETURN_META(MRES_IGNORED);
}

void LightStyle(int style, const char *val)
{
	BotSense::LightStyle(style, val);
	RETURN_META(MRES_IGNORED);
}

void PlaybackEvent(int flags, const edict_t *pInvoker, unsigned short eventindex, float delay,
	float *origin, float *angles, float fparam1, float fparam2, int iparam1, int iparam2,
	int bparam1, int bparam2)
{
	BotManager::OnEvent(flags, pInvoker);
	BrainFeed::OnPlaybackEvent(pInvoker, eventindex, iparam1);
	RETURN_META(MRES_IGNORED);
}

unsigned short PrecacheEvent(int type, const char *psz)
{
	BrainFeed::OnPrecacheEvent(psz, META_RESULT_ORIG_RET(unsigned short));
	RETURN_META_VALUE(MRES_IGNORED, 0);
}

void AlertMessage(ALERT_TYPE atype, const char *szFmt, ...)
{
	if (atype == at_logged && szFmt)
	{
		char line[1024];
		va_list args;
		va_start(args, szFmt);
		vsnprintf(line, sizeof(line), szFmt, args);
		va_end(args);
		BrainFeed::OnLog(line);
	}
	RETURN_META(MRES_IGNORED);
}

int DazeRegUserMsg(const char *pszName, int iSize)
{
	BotDaze::OnRegUserMsg(pszName, META_RESULT_ORIG_RET(int));
	BrainFeed::OnRegUserMsg(pszName, META_RESULT_ORIG_RET(int));
	RETURN_META_VALUE(MRES_IGNORED, 0);
}

void DazeMessageBegin(int msg_dest, int msg_type, const float *pOrigin, edict_t *ed)
{
	BotDaze::OnMessageBegin(msg_dest, msg_type, ed);
	BrainFeed::OnMessageBegin(msg_dest, msg_type, ed);
	RETURN_META(MRES_IGNORED);
}

void DazeWriteByte(int iValue)
{
	BotDaze::OnWrite(iValue);
	BrainFeed::OnWriteByte(iValue);
	RETURN_META(MRES_IGNORED);
}

void DazeWriteShort(int iValue)
{
	BotDaze::OnWrite(iValue);
	BrainFeed::OnWriteShort(iValue);
	RETURN_META(MRES_IGNORED);
}

void FeedWriteString(const char *sz)
{
	BrainFeed::OnWriteString(sz);
	RETURN_META(MRES_IGNORED);
}

void DazeMessageEnd()
{
	BotDaze::OnMessageEnd();
	BrainFeed::OnMessageEnd();
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
	pengfuncsFromEngine->pfnCmd_Args = FakeCmd::Args;
	pengfuncsFromEngine->pfnCmd_Argv = FakeCmd::Argv;
	pengfuncsFromEngine->pfnCmd_Argc = FakeCmd::Argc;
#ifdef CROSSBOT_PERF
	pengfuncsFromEngine->pfnTraceLine = PerfTraceLine;
	pengfuncsFromEngine->pfnTraceHull = PerfTraceHull;
	pengfuncsFromEngine->pfnTraceMonsterHull = PerfTraceMonsterHull;
#endif
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
	pengfuncsFromEngine->pfnLightStyle = LightStyle;
	pengfuncsFromEngine->pfnRegUserMsg = DazeRegUserMsg;
	pengfuncsFromEngine->pfnMessageBegin = DazeMessageBegin;
	pengfuncsFromEngine->pfnWriteByte = DazeWriteByte;
	pengfuncsFromEngine->pfnWriteShort = DazeWriteShort;
	pengfuncsFromEngine->pfnMessageEnd = DazeMessageEnd;
	pengfuncsFromEngine->pfnWriteString = FeedWriteString;
	pengfuncsFromEngine->pfnPrecacheEvent = PrecacheEvent;
	pengfuncsFromEngine->pfnAlertMessage = AlertMessage;
#ifdef CROSSBOT_PERF
	pengfuncsFromEngine->pfnTraceLine = PerfTraceLine_Post;
	pengfuncsFromEngine->pfnTraceHull = PerfTraceHull_Post;
	pengfuncsFromEngine->pfnTraceMonsterHull = PerfTraceMonsterHull_Post;
#endif
	return TRUE;
}
