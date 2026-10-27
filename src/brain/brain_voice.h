#pragma once

#include <string>

struct CBot;

namespace BrainVoice
{

void Attach();

void Detach();

void Frame();

void OnMapEnd();

void Tone(CBot &bot, float seconds, float hz);
void Echo(CBot *bot);
bool Speak(CBot &bot, const char *text, bool loopback, std::string &result);

void PrintStatus();

}
