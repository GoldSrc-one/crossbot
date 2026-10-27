#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "../game/crossplay.h"
#include "brain_config.h"
#include "json.h"

namespace
{
	BrainConfig::Config g_config;

	bool ReadFile(const char *path, std::string &out)
	{
		FILE *f = fopen(path, "rb");
		if (!f)
			return false;
		char buf[4096];
		size_t n = 0;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
			out.append(buf, n);
		fclose(f);
		return true;
	}

	void ReadLevels(const Json::Doc &doc, int obj, const char *name, std::vector<float> &out,
		std::initializer_list<float> def)
	{
		out.assign(def);
		int arr = doc.Member(obj, name);
		int count = doc.Count(arr);
		if (doc.TypeOf(arr) != Json::T_ARRAY || count < 2)
			return;
		std::vector<float> levels;
		for (int i = 0; i < count; ++i)
		{
			float v = 0.f;
			if (!doc.Num(doc.Element(arr, i), v))
				return;
			levels.push_back(v);
		}
		out.swap(levels);
	}

	std::string ReadText(const Json::Doc &doc, int obj, const char *name)
	{
		int tok = doc.Member(obj, name);
		if (doc.TypeOf(tok) != Json::T_ARRAY)
			return doc.Str(tok);
		std::string out;
		for (int i = 0; i < doc.Count(tok); ++i)
			out += doc.Str(doc.Element(tok, i)) + "\n";
		return out;
	}

	std::string ReadPrompts(const Json::Doc &doc, int prompts, BrainConfig::Config &c)
	{
		if (doc.TypeOf(prompts) != Json::T_OBJECT)
			return "brain.json has no prompts (copy them from brain.example.json)";
		int jev = doc.Member(prompts, "jev");
		int act = doc.Member(jev, "act");
		int involve = doc.Member(jev, "involve");
		int remember = doc.Member(jev, "remember");
		int ignore = doc.Member(jev, "ignore");
		int recheck = doc.Member(jev, "recheck");

		int tools = doc.Member(prompts, "tools");
		if (doc.TypeOf(tools) == Json::T_ARRAY)
			c.tools = doc.Raw(tools);
		int why = doc.Member(ignore, "why");
		if (doc.TypeOf(why) == Json::T_ARRAY)
		{
			for (int i = 0; i < doc.Count(why); ++i)
				c.ignoreWhy.push_back(doc.Str(doc.Element(why, i)));
		}

		struct Field { const char *name; std::string *out; int obj; const char *key; };
		const Field fields[] = {
			{ "system", &c.system, prompts, "system" },
			{ "tools", &c.tools, -1, nullptr },
			{ "gate", &c.gate, prompts, "gate" },
			{ "jev.act.instructions", &c.actAsk, act, "instructions" },
			{ "jev.act.yes", &c.actYes, act, "yes" },
			{ "jev.act.no", &c.actNo, act, "no" },
			{ "jev.involve.instructions", &c.involveAsk, involve, "instructions" },
			{ "jev.involve.yes", &c.involveYes, involve, "yes" },
			{ "jev.involve.no", &c.involveNo, involve, "no" },
			{ "jev.remember.instructions", &c.rememberAsk, remember, "instructions" },
			{ "jev.remember.yes", &c.rememberYes, remember, "yes" },
			{ "jev.remember.no", &c.rememberNo, remember, "no" },
			{ "jev.ignore.instructions", &c.ignoreAsk, ignore, "instructions" },
			{ "jev.ignore.none", &c.ignoreNone, ignore, "none" },
			{ "jev.ignore.level", &c.ignoreLevel, ignore, "level" },
			{ "jev.recheck.instructions", &c.recheckAsk, recheck, "instructions" },
			{ "jev.recheck.none", &c.recheckNone, recheck, "none" },
			{ "jev.recheck.level", &c.recheckLevel, recheck, "level" },
		};
		for (const Field &f : fields)
		{
			if (f.key)
				*f.out = ReadText(doc, f.obj, f.key);
			if (f.out->empty())
				return std::string("brain.json has no prompts.") + f.name;
		}
		if (c.ignoreWhy.empty())
			return "brain.json has no prompts.jev.ignore.why";
		return std::string();
	}
}

namespace BrainConfig
{

void Load()
{
	Config c;
	char path[512];
	snprintf(path, sizeof(path), "%s/addons/crossbot/brain.json", XPlay::HyperDir());

	std::string text;
	Json::Doc doc;
	if (!ReadFile(path, text))
	{
		c.why = "no addons/crossbot/brain.json";
	}
	else if (!doc.Parse(text) || doc.TypeOf(doc.Root()) != Json::T_OBJECT)
	{
		c.why = "brain.json is not a JSON object";
	}
	else
	{
		int root = doc.Root();
		int llm = doc.Member(root, "llm");
		int jev = doc.Member(root, "jev");
		int hist = doc.Member(root, "history");
		int rate = doc.Member(root, "rate");
		int voice = doc.Member(root, "voice");

		c.loaded = true;
		c.enabled = doc.GetBool(root, "enabled", true);
		c.key = doc.GetStr(root, "key");

		c.llmUrl = doc.GetStr(llm, "url", "https://openrouter.ai/api/v1/chat/completions");
		c.llmModel = doc.GetStr(llm, "model", "nvidia/nemotron-3-super-120b-a12b:free");
		c.llmTimeout = doc.GetNum(llm, "timeout", c.llmTimeout);
		c.llmMaxTokens = static_cast<int>(doc.GetNum(llm, "max_tokens", static_cast<float>(c.llmMaxTokens)));
		c.reasoning = doc.GetStr(llm, "reasoning", "off");

		c.jevUrl = doc.GetStr(jev, "url", "https://openrouter.ai/api/alpha/decisions");
		c.jevModel = doc.GetStr(jev, "model", "typesafe/jev-1.13");
		c.jevTimeout = doc.GetNum(jev, "timeout", c.jevTimeout);
		c.act = doc.GetNum(jev, "act", c.act);
		c.involve = doc.GetNum(jev, "involve", c.involve);
		c.remember = doc.GetNum(jev, "remember", c.remember);
		c.ignoreMin = doc.GetNum(jev, "ignore_min", c.ignoreMin);
		ReadLevels(doc, jev, "ignore_levels", c.ignoreLevels, { 0.f, 30.f, 120.f, 600.f, 3600.f });
		c.recheckMin = doc.GetNum(jev, "recheck_min", c.recheckMin);
		ReadLevels(doc, jev, "recheck_levels", c.recheckLevels, { 0.f, 5.f, 15.f, 30.f, 60.f, 180.f });
		c.recheckChain = static_cast<int>(doc.GetNum(jev, "recheck_chain", static_cast<float>(c.recheckChain)));

		c.events = static_cast<int>(doc.GetNum(hist, "events", static_cast<float>(c.events)));
		c.comms = static_cast<int>(doc.GetNum(hist, "comms", static_cast<float>(c.comms)));
		c.commsSeconds = doc.GetNum(hist, "comms_seconds", c.commsSeconds);
		c.memories = static_cast<int>(doc.GetNum(hist, "memories", static_cast<float>(c.memories)));

		c.linesPerMinute = static_cast<int>(doc.GetNum(rate, "lines_per_minute", static_cast<float>(c.linesPerMinute)));
		c.radioHold = doc.GetNum(rate, "radio_hold", c.radioHold);

		c.voice = doc.GetBool(voice, "enabled", c.voice);
		c.sttUrl = doc.GetStr(voice, "stt_url", "https://openrouter.ai/api/v1/audio/transcriptions");
		c.sttModel = doc.GetStr(voice, "stt_model", "openai/gpt-4o-mini-transcribe");
		c.sttTimeout = doc.GetNum(voice, "stt_timeout", c.sttTimeout);
		c.ttsUrl = doc.GetStr(voice, "tts_url", "https://openrouter.ai/api/v1/audio/speech");
		c.ttsModel = doc.GetStr(voice, "tts_model", "fish-audio/s2.1-pro-free:free");
		c.ttsVoice = doc.GetStr(voice, "tts_voice");
		c.ttsTimeout = doc.GetNum(voice, "tts_timeout", c.ttsTimeout);
		c.voiceMin = doc.GetNum(voice, "min_seconds", c.voiceMin);
		c.voiceMax = doc.GetNum(voice, "max_seconds", c.voiceMax);
		c.voiceSilence = doc.GetNum(voice, "silence", c.voiceSilence);
		c.voiceRate = static_cast<int>(doc.GetNum(voice, "send_rate", static_cast<float>(c.voiceRate)));

		std::string bad = ReadPrompts(doc, doc.Member(root, "prompts"), c);
		if (c.key.empty())
		{
			c.enabled = false;
			c.why = "brain.json has no key";
		}
		else if (!bad.empty())
		{
			c.enabled = false;
			c.why = bad;
		}
		else if (!c.enabled)
		{
			c.why = "disabled in brain.json";
		}
	}

	g_config = c;
}

const Config &Get()
{
	return g_config;
}

float LevelSeconds(const std::vector<float> &levels, float score)
{
	if (levels.empty())
		return 0.f;
	if (score <= 0.f)
		return levels.front();
	float last = static_cast<float>(levels.size() - 1);
	if (score >= last)
		return levels.back();
	size_t i = static_cast<size_t>(floorf(score));
	float frac = score - static_cast<float>(i);
	return levels[i] + (levels[i + 1] - levels[i]) * frac;
}

void Print()
{
	const Config &c = g_config;
	char line[512];
	snprintf(line, sizeof(line),
		"CrossBot: brain config %s%s%s, key %s, jev %s @ %s (%.0fs), llm %s @ %s (%.0fs, %d tokens, reasoning %s)\n",
		c.loaded ? "loaded" : "missing", c.enabled ? ", enabled" : ", disabled: ", c.enabled ? "" : c.why.c_str(),
		c.key.empty() ? "empty" : "set", c.jevModel.c_str(), c.jevUrl.c_str(), c.jevTimeout, c.llmModel.c_str(),
		c.llmUrl.c_str(), c.llmTimeout, c.llmMaxTokens, c.reasoning.c_str());
	SERVER_PRINT(line);
	snprintf(line, sizeof(line),
		"CrossBot: brain thresholds act %.2f involve %.2f remember %.2f ignore>=%.0fs recheck>=%.0fs chain %d, "
		"history %d events / %d comms / %.0fs / %d memories, %d lines/min, radio hold %.1fs\n",
		c.act, c.involve, c.remember, c.ignoreMin, c.recheckMin, c.recheckChain, c.events, c.comms, c.commsSeconds,
		c.memories, c.linesPerMinute, c.radioHold);
	SERVER_PRINT(line);
	snprintf(line, sizeof(line), "CrossBot: brain voice %s, stt %s, tts %s%s%s, utterances %.1f-%.0fs after %.1fs silence, send %d Hz\n",
		c.voice ? "on" : "off", c.sttModel.c_str(), c.ttsModel.c_str(), c.ttsVoice.empty() ? "" : " voice ",
		c.ttsVoice.c_str(), c.voiceMin, c.voiceMax, c.voiceSilence, c.voiceRate);
	SERVER_PRINT(line);
}

}
