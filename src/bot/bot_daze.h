#pragma once

#include <extdll.h>

struct CBot;

namespace BotDaze
{

struct State
{
	float blindStart = 0.f;
	float blindUntil = 0.f;
	float fadeSec = 0.f;
	float holdSec = 0.f;
	int alpha = 0;
	unsigned int flashes = 0;

	int concuss = 0;
	float concussStart = 0.f;
	unsigned int concussUpdates = 0;
	unsigned int concs = 0;
};

bool Blind(const CBot &bot);

void Sway(const CBot &bot, Vector &angles);

void OnScreenFade(CBot &bot, int duration, int hold, int flags, int r, int g, int b, int a);

void OnConcuss(CBot &bot, int scale);

void OnRegUserMsg(const char *name, int id);
void OnMessageBegin(int dest, int type, edict_t *ed);
void OnWrite(int value);
void OnMessageEnd();

void PrintStatus(const CBot &bot);
void PrintUserMsgs();

}
