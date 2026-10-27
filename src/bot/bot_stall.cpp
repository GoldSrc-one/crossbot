#include <extdll.h>
#include <meta_api.h>

#include "bot_stall.h"
#include "../game/cvars.h"

#if defined(CROSSBOT_PERF) && defined(_WIN32)

#include <windows.h>
#include <psapi.h>

#pragma comment(lib, "psapi.lib")

#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>

namespace
{
	constexpr int kMaxModules = 256;
	constexpr int kMaxFrames = 24;
	constexpr int kRing = 32;
	constexpr int kMaxSamplesPerStall = 8;
	constexpr DWORD kStackScanBytes = 32 * 1024;

	struct Module
	{
		DWORD base;
		DWORD size;
		char name[32];
	};

	struct Sample
	{
		int stallMs;
		bool inCrossbot;
		int count;
		DWORD addr[kMaxFrames];
	};

	cvar_t g_stallCvar = { "crossbot_hitch_stack", "0", FCVAR_SERVER };
	cvar_t *g_stall = nullptr;

	HANDLE g_mainThread = nullptr;
	DWORD g_stackBase = 0;
	LARGE_INTEGER g_freq;

	Module g_modules[kMaxModules];
	int g_moduleCount = 0;

	std::atomic<long long> g_beat{ 0 };
	std::atomic<bool> g_inCrossbot{ false };
	std::atomic<int> g_thresholdMs{ 0 };
	std::atomic<bool> g_quit{ false };
	std::thread g_worker;

	Sample g_ring[kRing];
	std::atomic<int> g_written{ 0 };
	int g_read = 0;

	long long Now()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return li.QuadPart;
	}

	void SnapshotModules()
	{
		HMODULE handles[kMaxModules];
		DWORD needed = 0;
		HANDLE self = GetCurrentProcess();
		if (!EnumProcessModules(self, handles, sizeof(handles), &needed))
			return;
		int count = static_cast<int>(needed / sizeof(HMODULE));
		if (count > kMaxModules)
			count = kMaxModules;
		int out = 0;
		for (int i = 0; i < count; ++i)
		{
			MODULEINFO info;
			if (!GetModuleInformation(self, handles[i], &info, sizeof(info)))
				continue;
			char path[MAX_PATH];
			if (!GetModuleFileNameA(handles[i], path, sizeof(path)))
				continue;
			const char *slash = strrchr(path, '\\');
			Module &m = g_modules[out++];
			m.base = static_cast<DWORD>(reinterpret_cast<uintptr_t>(info.lpBaseOfDll));
			m.size = info.SizeOfImage;
			snprintf(m.name, sizeof(m.name), "%s", slash ? slash + 1 : path);
		}
		g_moduleCount = out;
	}

	const Module *ModuleOf(DWORD addr)
	{
		for (int i = 0; i < g_moduleCount; ++i)
		{
			if (addr >= g_modules[i].base && addr - g_modules[i].base < g_modules[i].size)
				return &g_modules[i];
		}
		return nullptr;
	}

	void Capture(int stallMs)
	{
		int written = g_written.load(std::memory_order_relaxed);
		if (written - g_read >= kRing)
			return;
		Sample &s = g_ring[written % kRing];
		s.stallMs = stallMs;
		s.inCrossbot = g_inCrossbot.load();
		s.count = 0;

		if (SuspendThread(g_mainThread) == static_cast<DWORD>(-1))
			return;

		CONTEXT ctx;
		memset(&ctx, 0, sizeof(ctx));
		ctx.ContextFlags = CONTEXT_CONTROL;
		if (GetThreadContext(g_mainThread, &ctx))
		{
			s.addr[s.count++] = ctx.Eip;
			DWORD sp = ctx.Esp;
			DWORD end = sp + kStackScanBytes;
			if (g_stackBase && end > g_stackBase)
				end = g_stackBase;
			for (DWORD p = sp; p + 4 <= end && s.count < kMaxFrames; p += 4)
			{
				DWORD v = *reinterpret_cast<const DWORD *>(static_cast<uintptr_t>(p));
				if (ModuleOf(v))
					s.addr[s.count++] = v;
			}
		}

		ResumeThread(g_mainThread);
		g_written.store(written + 1, std::memory_order_release);
	}

	void Work()
	{
		long long seenBeat = 0;
		int taken = 0;
		long long nextAt = 0;
		while (!g_quit.load())
		{
			Sleep(2);
			int threshold = g_thresholdMs.load();
			long long beat = g_beat.load();
			if (threshold <= 0 || beat == 0)
				continue;
			if (beat != seenBeat)
			{
				seenBeat = beat;
				taken = 0;
				nextAt = 0;
			}
			long long now = Now();
			int stallMs = static_cast<int>((now - beat) * 1000 / g_freq.QuadPart);
			if (stallMs < threshold || taken >= kMaxSamplesPerStall || now < nextAt)
				continue;
			Capture(stallMs);
			++taken;
			int gap = threshold / 2 > 20 ? threshold / 2 : 20;
			nextAt = now + g_freq.QuadPart * gap / 1000;
		}
	}
}

namespace BotStall
{

void Register()
{
	g_engfuncs.pfnCvar_RegisterVariable(&g_stallCvar);
	g_stall = CBotCvar("crossbot_hitch_stack");
	QueryPerformanceFrequency(&g_freq);
	DuplicateHandle(GetCurrentProcess(), GetCurrentThread(), GetCurrentProcess(), &g_mainThread,
		THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, 0);
	NT_TIB *tib = reinterpret_cast<NT_TIB *>(NtCurrentTeb());
	g_stackBase = static_cast<DWORD>(reinterpret_cast<uintptr_t>(tib->StackBase));
}

void Beat(bool inCrossbot)
{
	g_inCrossbot.store(inCrossbot);
	g_beat.store(Now());

	int want = g_stall ? static_cast<int>(g_stall->value) : 0;
	if (want > 0 && !g_worker.joinable() && g_mainThread)
	{
		SnapshotModules();
		g_quit.store(false);
		g_worker = std::thread(Work);
	}
	if (want != g_thresholdMs.load())
	{
		if (want > 0)
			SnapshotModules();
		g_thresholdMs.store(want);
	}
}

void Flush()
{
	int written = g_written.load(std::memory_order_acquire);
	while (g_read < written)
	{
		const Sample &s = g_ring[g_read % kRing];
		char line[1400];
		int len = snprintf(line, sizeof(line), "CrossBot HITCH STACK stalled %dms %s:", s.stallMs,
			s.inCrossbot ? "inside crossbot StartFrame" : "outside crossbot");
		for (int i = 0; i < s.count && len < static_cast<int>(sizeof(line)) - 64; ++i)
		{
			const Module *m = ModuleOf(s.addr[i]);
			if (m)
				len += snprintf(line + len, sizeof(line) - len, " %s+0x%lx", m->name,
					static_cast<unsigned long>(s.addr[i] - m->base));
			else
				len += snprintf(line + len, sizeof(line) - len, " 0x%08lx", static_cast<unsigned long>(s.addr[i]));
		}
		snprintf(line + len, sizeof(line) - len, "\n");
		SERVER_PRINT(line);
		++g_read;
	}
}

void Shutdown()
{
	g_quit.store(true);
	if (g_worker.joinable())
		g_worker.join();
}

}

#endif
