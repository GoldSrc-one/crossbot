#pragma once

#include <string>
#include <vector>

namespace BrainConfig
{

struct Config
{
	bool loaded = false;
	bool enabled = false;
	std::string why;
	std::string key;

	std::string llmUrl;
	std::string llmModel;
	float llmTimeout = 20.f;
	int llmMaxTokens = 1500;
	std::string reasoning;

	std::string jevUrl;
	std::string jevModel;
	float jevTimeout = 5.f;
	float act = 0.5f;
	float involve = 0.5f;
	float remember = 0.7f;
	float ignoreMin = 30.f;
	std::vector<float> ignoreLevels;
	float recheckMin = 10.f;
	std::vector<float> recheckLevels;
	int recheckChain = 2;

	int events = 30;
	int comms = 20;
	float commsSeconds = 180.f;
	int memories = 20;

	int linesPerMinute = 6;
	float radioHold = 1.f;

	bool voice = true;
	std::string sttUrl;
	std::string sttModel;
	float sttTimeout = 15.f;
	std::string ttsUrl;
	std::string ttsModel;
	std::string ttsVoice;
	float ttsTimeout = 30.f;
	float voiceMin = 0.5f;
	float voiceMax = 15.f;
	float voiceSilence = 0.6f;
	int voiceRate = 24000;

	std::string system;
	std::string tools;
	std::string gate;
	std::string actAsk, actYes, actNo;
	std::string involveAsk, involveYes, involveNo;
	std::string rememberAsk, rememberYes, rememberNo;
	std::string ignoreAsk, ignoreNone, ignoreLevel;
	std::vector<std::string> ignoreWhy;
	std::string recheckAsk, recheckNone, recheckLevel;
};

void Load();

const Config &Get();

float LevelSeconds(const std::vector<float> &levels, float score);

void Print();

}
