#pragma once

namespace BotPerf
{
	enum Zone
	{
		ZONE_OUTSIDE = 0,
		ZONE_FRAME,
		ZONE_BEAMS,
		ZONE_RESPAWN,
		ZONE_WEAPONS,
		ZONE_MOVENAV,
		ZONE_FINDAREA,
		ZONE_SENSE,
		ZONE_SN_VANTAGE,
		ZONE_SN_DAMAGE,
		ZONE_SN_VISION,
		ZONE_SN_BONE,
		ZONE_SN_LOS,
		ZONE_SN_ENEMY,
		ZONE_SN_LOOK,
		ZONE_FIRE,
		ZONE_COMBAT,
		ZONE_MEMOS,
		ZONE_BREAK,
		ZONE_DEPLOY_TRY,
		ZONE_DEPLOY_RUN,
		ZONE_ITEM_GOAL,
		ZONE_ITEM_SELECT,
		ZONE_REPATH,
		ZONE_RP_PLAN,
		ZONE_RP_PROFILE,
		ZONE_RP_LOG,
		ZONE_PLANTRACES,
		ZONE_PUSHFIELDS,
		ZONE_FOLLOW,
		ZONE_AVOID,
		ZONE_OPPO,
		ZONE_LOOK,
		ZONE_DRIVE,
		ZONE_RUNMOVE,
		ZONE_TRACELOG,
		ZONE_COUNT
	};

	enum TraceType
	{
		TRACE_LINE_T = 0,
		TRACE_HULL_T,
		TRACE_MODEL_T,
		TRACE_MONSTER_T,
		TRACE_TYPE_COUNT
	};

#ifdef CROSSBOT_PERF

	bool Enabled();

	void FrameBegin();
	void FrameEnd();

	void Enter(Zone z);
	void Leave(Zone z);

	void NoteBot(const char *name);

	void InstallTraceWrappers();

	void TraceBegin(TraceType type, const float *v1, const float *v2, int fNoMonsters);
	void TraceEnd();

	void Status();
	void Reset();

	struct Scope
	{
		Zone zone;
		bool on;

		explicit Scope(Zone z) : zone(z), on(Enabled())
		{
			if (on)
				Enter(zone);
		}

		~Scope()
		{
			if (on)
				Leave(zone);
		}
	};

#else

	inline bool Enabled() { return false; }

	inline void FrameBegin() {}
	inline void FrameEnd() {}

	inline void Enter(Zone) {}
	inline void Leave(Zone) {}

	inline void NoteBot(const char *) {}

	inline void InstallTraceWrappers() {}

	inline void TraceBegin(TraceType, const float *, const float *, int) {}
	inline void TraceEnd() {}

	inline void Status() {}
	inline void Reset() {}

	struct Scope
	{
		explicit Scope(Zone) {}
	};

#endif
}

#define BOTPERF_CAT2(a, b) a##b
#define BOTPERF_CAT(a, b) BOTPERF_CAT2(a, b)
#define BOTPERF_SCOPE(z) BotPerf::Scope BOTPERF_CAT(botPerfScope, __LINE__)(BotPerf::z)
