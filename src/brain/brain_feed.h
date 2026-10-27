#pragma once

#include <extdll.h>

#include <deque>

namespace BrainFeed
{

enum Kind : unsigned char
{
	EV_CHAT = 0,
	EV_RADIO,
	EV_KILL,
	EV_OBJECTIVE,
	EV_ROUND,
	EV_SCORE,
	EV_PRESENCE,
	EV_VOICE,
	EV_COUNT,
};

constexpr int kNameLength = 32;
constexpr int kTextLength = 192;

struct Event
{
	unsigned int serial = 0;
	float time = 0.f;
	Kind kind = EV_CHAT;
	bool teamOnly = false;
	bool human = false;
	bool dead = false;
	bool bot = false;
	bool automatic = false;
	int team = 0;
	int game = -1;
	int userid = 0;
	int entIndex = 0;
	char who[kNameLength] = {};
	char text[kTextLength] = {};
};

void Reset();

void OnRegUserMsg(const char *name, int id);

void OnMessageBegin(int dest, int type, edict_t *ed);

void OnWriteByte(int value);

void OnWriteShort(int value);

void OnWriteString(const char *value);

void OnMessageEnd();

void OnPrecacheEvent(const char *name, unsigned short index);

void OnPlaybackEvent(const edict_t *invoker, unsigned short eventindex, int iparam1);

void OnClientCommand(edict_t *ent);

void NoteChat(edict_t *ent, bool team, const char *text, Kind kind = EV_CHAT);

bool IsComms(Kind kind);

bool IsHuman(const edict_t *ent);

void OnLog(const char *line);

bool TakeRoundStart();

int Round();

float RoundTime();

unsigned int RoundSerial();

const std::deque<Event> &Events();

unsigned int LastSerial();

const char *KindName(Kind kind);

void Print(int count);

}
