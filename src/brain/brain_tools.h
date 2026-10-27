#pragma once

#include <extdll.h>

#include <string>

#include "../bot/bot_directive.h"
#include "json.h"

struct CBot;

namespace BrainTools
{

bool AimPoint(edict_t *player, Vector &out);

edict_t *FindPlayer(const char *name);

bool ResolveTarget(const char *ref, edict_t *speaker, BotDirective::Target &out, std::string &err);

bool Execute(CBot &bot, const char *tool, const Json::Doc &doc, int args, edict_t *speaker, std::string &result);

std::string Sanitize(const char *text, size_t maxLength);

}
