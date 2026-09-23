#pragma once

#include <extdll.h>

#include <type_traits>

#if defined(_WIN32)
#define CB_STRICMP _stricmp
#define CB_STRNICMP _strnicmp
#else
#define CB_STRICMP strcasecmp
#define CB_STRNICMP strncasecmp
#endif

class IGameProfile;

namespace XPlay
{

constexpr int kMaxGames = 8;

int NumGames();
const char *GameDir(int iGame);
int FindGame(const char *dir);

int MapOwner();
int OwnerOf(const edict_t *ent);

void *PrivateData(const edict_t *ent, int iGame);

const char *HyperDir();
bool FindMapBsp(const char *mapName, char *out, unsigned int size);

int Current();
void EnterGame(int iGame);
void LeaveGame();

struct ScopedGame
{
	explicit ScopedGame(int iGame) : prev(Current()) { EnterGame(iGame); }
	explicit ScopedGame(const edict_t *ent) : prev(Current()) { EnterGame(OwnerOf(ent)); }
	~ScopedGame() { EnterGame(prev); }
	ScopedGame(const ScopedGame &) = delete;
	ScopedGame &operator=(const ScopedGame &) = delete;
	int prev;
};

bool Supported(int iGame);
int PickRandomSupported();
void DetectProfiles();
void ForgetProfiles();

IGameProfile *ProfileFor(int iGame);

void RunInGame(int iGame, void (*callback)(void *context), void *context);

template <typename F>
inline void InGame(int iGame, F &&fn)
{
	using Fn = typename std::remove_reference<F>::type;

	Fn *pfn = &fn;
	RunInGame(iGame, [](void *ctx) { (*static_cast<Fn *>(ctx))(); }, pfn);
}

}
