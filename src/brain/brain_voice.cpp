#include <extdll.h>
#include <meta_api.h>

#include <opus.h>

#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../bot/bot.h"
#include "../game/cvars.h"
#include "../game/crossplay.h"
#include "../game/teams.h"
#include "../meta/ex_rehlds_api.h"
#include "brain.h"
#include "brain_config.h"
#include "brain_feed.h"
#include "brain_http.h"
#include "brain_tools.h"
#include "brain_voice.h"
#include "crc32.h"
#include "json.h"

namespace
{
	constexpr int kClcVoiceData = 8;
	constexpr int kSvcVoiceData = 53;
	constexpr int kMaxPlayers = 32;
	constexpr int kListenRate = 16000;
	constexpr int kListenFrame = kListenRate / 50;
	constexpr int kMaxDecode = 5760;
	constexpr int kMaxLost = 10;
	constexpr int kLookahead = 5;
	constexpr int kFramesPerPacket = 5;
	constexpr int kMaxPacket = 2048;
	constexpr size_t kMaxSpeech = 200;
	constexpr int kWorkers = 3;

	enum Op : unsigned char
	{
		OP_SILENCE = 0,
		OP_SILK = 4,
		OP_OPUS = 5,
		OP_OPUS_PLC = 6,
		OP_RATE = 11,
	};

	struct Listener
	{
		int userid = 0;
		OpusDecoder *decoder = nullptr;
		int seq = -1;
		std::vector<short> pcm;
		float last = 0.f;
		int rate = 0;
		const char *codec = "none";
		unsigned int packets = 0;
		unsigned int rejected = 0;
		unsigned int utterances = 0;
		bool announced = false;
	};

	struct Talk
	{
		int userid = 0;
		bool loopback = false;
		int rate = 24000;
		std::deque<std::string> frames;
		unsigned short seq = 0;
		float start = -1.f;
		int sent = 0;
		int receivers = 0;
	};

	Listener g_listen[kMaxPlayers];
	Talk g_talk[kMaxPlayers];
	bool g_hooked = false;

	std::mutex g_mutex;
	std::condition_variable g_wake;
	std::deque<std::function<void()>> g_work;
	std::deque<std::function<void()>> g_done;
	std::vector<std::thread> g_threads;
	bool g_quit = false;

	unsigned int g_sttCalls = 0;
	unsigned int g_ttsCalls = 0;
	float g_sttSeconds = 0.f;
	float g_ttsSeconds = 0.f;
	std::string g_lastError;

	bool Debug()
	{
		cvar_t *cvar = CBotCvar("crossbot_brain_debug");
		return cvar && cvar->value != 0.f;
	}

	void Loop()
	{
		for (;;)
		{
			std::function<void()> fn;
			{
				std::unique_lock<std::mutex> lock(g_mutex);
				g_wake.wait(lock, [] { return g_quit || !g_work.empty(); });
				if (g_quit)
					return;
				fn = std::move(g_work.front());
				g_work.pop_front();
			}
			fn();
		}
	}

	void Async(std::function<void()> fn)
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		if (g_threads.empty())
		{
			g_quit = false;
			for (int i = 0; i < kWorkers; ++i)
				g_threads.emplace_back(Loop);
		}
		g_work.push_back(std::move(fn));
		g_wake.notify_one();
	}

	void Done(std::function<void()> fn)
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_done.push_back(std::move(fn));
	}

	void Put16(std::string &out, unsigned int v)
	{
		out += static_cast<char>(v & 0xFF);
		out += static_cast<char>((v >> 8) & 0xFF);
	}

	void Put32(std::string &out, uint32_t v)
	{
		Put16(out, v & 0xFFFF);
		Put16(out, v >> 16);
	}

	unsigned int Get16(const unsigned char *p)
	{
		return p[0] | (p[1] << 8);
	}

	uint32_t Get32(const unsigned char *p)
	{
		return Get16(p) | (static_cast<uint32_t>(Get16(p + 2)) << 16);
	}

	std::string Wav(const std::vector<short> &pcm, int rate)
	{
		std::string out = "RIFF";
		uint32_t bytes = static_cast<uint32_t>(pcm.size() * 2);
		Put32(out, 36 + bytes);
		out += "WAVEfmt ";
		Put32(out, 16);
		Put16(out, 1);
		Put16(out, 1);
		Put32(out, rate);
		Put32(out, rate * 2);
		Put16(out, 2);
		Put16(out, 16);
		out += "data";
		Put32(out, bytes);
		out.append(reinterpret_cast<const char *>(pcm.data()), bytes);
		return out;
	}

	std::string Base64(const std::string &in)
	{
		static const char kChars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		std::string out;
		out.reserve((in.size() + 2) / 3 * 4);
		size_t i = 0;
		for (; i + 2 < in.size(); i += 3)
		{
			uint32_t v = (static_cast<unsigned char>(in[i]) << 16) | (static_cast<unsigned char>(in[i + 1]) << 8) |
				static_cast<unsigned char>(in[i + 2]);
			out += kChars[(v >> 18) & 63];
			out += kChars[(v >> 12) & 63];
			out += kChars[(v >> 6) & 63];
			out += kChars[v & 63];
		}
		if (i < in.size())
		{
			uint32_t v = static_cast<unsigned char>(in[i]) << 16;
			if (i + 1 < in.size())
				v |= static_cast<unsigned char>(in[i + 1]) << 8;
			out += kChars[(v >> 18) & 63];
			out += kChars[(v >> 12) & 63];
			out += i + 1 < in.size() ? kChars[(v >> 6) & 63] : '=';
			out += '=';
		}
		return out;
	}

	int TypeParam(const std::string &type, const char *name, int def)
	{
		size_t at = type.find(name);
		if (at == std::string::npos)
			return def;
		int v = atoi(type.c_str() + at + strlen(name));
		return v > 0 ? v : def;
	}

	std::vector<short> Resample(const short *in, size_t count, int channels, int from, int to)
	{
		std::vector<short> out;
		size_t frames = count / channels;
		if (frames == 0 || from <= 0 || to <= 0)
			return out;
		size_t n = static_cast<size_t>(static_cast<double>(frames) * to / from);
		out.resize(n);
		for (size_t i = 0; i < n; ++i)
		{
			double pos = static_cast<double>(i) * from / to;
			size_t a = static_cast<size_t>(pos);
			size_t b = a + 1 < frames ? a + 1 : a;
			float t = static_cast<float>(pos - a);
			float va = 0.f;
			float vb = 0.f;
			for (int c = 0; c < channels; ++c)
			{
				va += in[a * channels + c];
				vb += in[b * channels + c];
			}
			out[i] = static_cast<short>((va + (vb - va) * t) / channels);
		}
		return out;
	}

	void Trace(const std::string &line)
	{
		std::string dir = Brain::TraceDir();
		if (dir.empty())
			return;
		std::string path = dir + "/voice_" + STRING(gpGlobals->mapname) + ".jsonl";
		FILE *f = fopen(path.c_str(), "ab");
		if (!f)
			return;
		fwrite(line.data(), 1, line.size(), f);
		fputc('\n', f);
		fclose(f);
	}

	edict_t *PlayerAt(int index, int userid)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(index + 1);
		if (!CBotIsLivePlayer(ent) || g_engfuncs.pfnGetPlayerUserId(ent) != userid)
			return nullptr;
		return ent;
	}

	bool TeamOnly(int)
	{
		cvar_t *allTalk = CBotCvar("sv_alltalk");
		return Teams::Enabled() && !(allTalk && allTalk->value != 0.f);
	}

	bool CanHear(edict_t *receiver, int sender)
	{
		return !TeamOnly(sender) || Teams::TeamOf(receiver) == Teams::TeamOf(g_engfuncs.pfnPEntityOfEntIndex(sender + 1));
	}

	void ResetListener(Listener &l)
	{
		if (l.decoder)
			opus_decoder_destroy(l.decoder);
		l = Listener();
	}

	void DecodeFrame(Listener &l, const unsigned char *data, int size)
	{
		short out[kMaxDecode];
		int n = opus_decode(l.decoder, data, size, out, data ? kMaxDecode : kListenFrame, 0);
		if (n > 0)
			l.pcm.insert(l.pcm.end(), out, out + n);
	}

	bool DecodePlc(Listener &l, const unsigned char *p, const unsigned char *end, bool &final)
	{
		while (p + 2 <= end)
		{
			unsigned int size = Get16(p);
			p += 2;
			if (size == 0xFFFF)
			{
				opus_decoder_ctl(l.decoder, OPUS_RESET_STATE);
				l.seq = -1;
				final = true;
				return true;
			}
			if (p + 2 > end)
				return false;
			int seq = static_cast<int>(Get16(p));
			p += 2;
			if (l.seq >= 0 && seq != l.seq)
			{
				int lost = (seq - l.seq) & 0xFFFF;
				if (lost > kMaxLost)
				{
					opus_decoder_ctl(l.decoder, OPUS_RESET_STATE);
					lost = 0;
				}
				for (int i = 0; i < lost; ++i)
					DecodeFrame(l, nullptr, 0);
			}
			l.seq = (seq + 1) & 0xFFFF;
			if (p + size > end)
				return false;
			if (size == 0)
				l.pcm.insert(l.pcm.end(), kListenFrame, 0);
			else
				DecodeFrame(l, p, static_cast<int>(size));
			p += size;
		}
		return true;
	}

	void Finish(int index);

	void Receive(int index, const unsigned char *data, int len)
	{
		Listener &l = g_listen[index];
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(index + 1);
		if (!CBotIsLivePlayer(ent))
			return;
		int userid = g_engfuncs.pfnGetPlayerUserId(ent);
		if (l.userid != userid)
		{
			ResetListener(l);
			l.userid = userid;
		}
		++l.packets;

		if (len < 13 || navgen::Crc32(data, len - 4) != Get32(data + len - 4))
		{
			++l.rejected;
			l.codec = "not steam voice";
			return;
		}
		if (!l.decoder)
		{
			int err = 0;
			l.decoder = opus_decoder_create(kListenRate, 1, &err);
			if (!l.decoder)
				return;
		}

		const unsigned char *p = data + 8;
		const unsigned char *end = data + len - 4;
		bool final = false;
		while (p < end)
		{
			unsigned char op = *p++;
			if (op == OP_RATE && p + 2 <= end)
			{
				l.rate = static_cast<int>(Get16(p));
				p += 2;
			}
			else if (op == OP_SILENCE && p + 2 <= end)
			{
				p += 2;
			}
			else if (op == OP_OPUS_PLC && p + 2 <= end)
			{
				unsigned int size = Get16(p);
				p += 2;
				const unsigned char *stop = p + size > end ? end : p + size;
				l.codec = "steam opus";
				if (!DecodePlc(l, p, stop, final))
					++l.rejected;
				p = stop;
			}
			else
			{
				l.codec = op == OP_SILK ? "steam silk (not decoded)" : (op == OP_OPUS ? "steam opus without plc (not decoded)" : "unknown opcode");
				++l.rejected;
				break;
			}
		}

		if (!l.announced)
		{
			l.announced = true;
			char line[192];
			snprintf(line, sizeof(line), "CrossBot: voice from %s: %s, %d Hz, %d bytes\n", CBotPlayerNameOf(ent), l.codec,
				l.rate, len);
			SERVER_PRINT(line);
		}
		l.last = gpGlobals->time;
		const BrainConfig::Config &config = BrainConfig::Get();
		if (final || l.pcm.size() >= static_cast<size_t>(config.voiceMax * kListenRate))
			Finish(index);
	}

	void Finish(int index)
	{
		Listener &l = g_listen[index];
		std::vector<short> pcm;
		pcm.swap(l.pcm);
		const BrainConfig::Config &config = BrainConfig::Get();
		float seconds = static_cast<float>(pcm.size()) / kListenRate;
		edict_t *ent = PlayerAt(index, l.userid);
		if (!ent || seconds < config.voiceMin || !config.enabled || !config.voice)
			return;
		int userid = l.userid;
		bool team = TeamOnly(index);
		unsigned int n = ++l.utterances;

		std::string wav = Wav(pcm, kListenRate);
		std::string dir = Brain::TraceDir();
		std::string wavPath;
		if (!dir.empty())
		{
			char name[96];
			snprintf(name, sizeof(name), "/voice_%s_%d_%u.wav", STRING(gpGlobals->mapname), userid, n);
			wavPath = dir + name;
			FILE *f = fopen(wavPath.c_str(), "wb");
			if (f)
			{
				fwrite(wav.data(), 1, wav.size(), f);
				fclose(f);
			}
		}

		std::string body = "{\"model\":" + Json::Quote(config.sttModel.c_str()) + ",\"input_audio\":{\"data\":\"" +
			Base64(wav) + "\",\"format\":\"wav\"}}";
		BrainConfig::Config cfg = config;
		Async([=] {
			BrainHttp::Response r;
			bool ok = BrainHttp::Post(cfg.sttUrl, cfg.key, body, cfg.sttTimeout, r);
			std::string text;
			Json::Doc doc;
			if (ok && doc.Parse(r.body))
				text = doc.GetStr(doc.Root(), "text");
			char head[256];
			snprintf(head, sizeof(head), "{\"time\":%lld,\"kind\":\"stt\",\"status\":%d,\"seconds\":%.3f,\"audio\":%.2f,\"model\":",
				static_cast<long long>(time(nullptr)), r.status, r.seconds, seconds);
			std::string trace = std::string(head) + Json::Quote(cfg.sttModel.c_str()) + ",\"wav\":" +
				Json::Quote(wavPath.c_str()) + ",\"error\":" + Json::Quote(r.error.c_str()) + ",\"response\":" +
				Json::Quote(r.body.substr(0, 2000).c_str()) + "}";
			Done([=] {
				Trace(trace);
				++g_sttCalls;
				g_sttSeconds = r.seconds;
				char line[320];
				if (!ok)
				{
					g_lastError = "stt http " + std::to_string(r.status) + " " + r.error + " " + r.body.substr(0, 160);
					snprintf(line, sizeof(line), "CrossBot: voice stt failed: %s\n", g_lastError.c_str());
					SERVER_PRINT(line);
					return;
				}
				std::string clean = BrainTools::Sanitize(text.c_str(), BrainFeed::kTextLength - 1);
				edict_t *who = PlayerAt(index, userid);
				snprintf(line, sizeof(line), "CrossBot: voice %s (%.1fs, stt %.2fs): %s\n", who ? CBotPlayerNameOf(who) : "?",
					seconds, r.seconds, clean.empty() ? "(nothing heard)" : clean.c_str());
				SERVER_PRINT(line);
				if (who && !clean.empty())
					BrainFeed::NoteChat(who, team, clean.c_str(), BrainFeed::EV_VOICE);
			});
		});
	}

	std::string Packet(Talk &t, bool final)
	{
		std::string payload;
		int count = 0;
		while (!t.frames.empty() && count < kFramesPerPacket)
		{
			const std::string &f = t.frames.front();
			Put16(payload, static_cast<unsigned int>(f.size()));
			Put16(payload, t.seq++);
			payload += f;
			t.frames.pop_front();
			++count;
		}
		t.sent += count;
		if (final && t.frames.empty())
			Put16(payload, 0xFFFF);

		std::string out;
		Put32(out, 0x00000011);
		Put32(out, 0x01100001);
		out += static_cast<char>(OP_RATE);
		Put16(out, static_cast<unsigned int>(t.rate));
		out += static_cast<char>(OP_OPUS_PLC);
		Put16(out, static_cast<unsigned int>(payload.size()));
		out += payload;
		Put32(out, navgen::Crc32(out.data(), out.size()));
		return out;
	}

	int Send(int sender, const std::string &packet)
	{
		if (!g_hooked)
			return 0;
		int receivers = 0;
		int maxClients = g_RehldsSvs->GetMaxClients();
		for (int i = 0; i < maxClients && i < kMaxPlayers; ++i)
		{
			if (i == sender)
				continue;
			IGameClient *cl = g_RehldsSvs->GetClient(i);
			if (!cl || !cl->IsActive())
				continue;
			edict_t *ent = cl->GetEdict();
			if (!CBotIsLivePlayer(ent) || !BrainFeed::IsHuman(ent))
				continue;
			if (!CanHear(ent, sender))
				continue;
			sizebuf_t *dg = cl->GetDatagram();
			if (dg->cursize + static_cast<int>(packet.size()) + 6 >= dg->maxsize)
				continue;
			g_RehldsFuncs->MSG_WriteByte(dg, kSvcVoiceData);
			g_RehldsFuncs->MSG_WriteByte(dg, sender);
			g_RehldsFuncs->MSG_WriteShort(dg, static_cast<int>(packet.size()));
			g_RehldsFuncs->MSG_WriteBuf(dg, static_cast<int>(packet.size()), const_cast<char *>(packet.data()));
			++receivers;
		}
		return receivers;
	}

	void Report(int sender)
	{
		if (!g_hooked)
		{
			SERVER_PRINT("CrossBot: voice report: no ReHLDS hook\n");
			return;
		}
		int maxClients = g_RehldsSvs->GetMaxClients();
		for (int i = 0; i < maxClients && i < kMaxPlayers; ++i)
		{
			IGameClient *cl = g_RehldsSvs->GetClient(i);
			if (i == sender || !cl || !cl->IsConnected())
				continue;
			edict_t *ent = cl->GetEdict();
			sizebuf_t *dg = cl->GetDatagram();
			char line[256];
			snprintf(line, sizeof(line), "CrossBot: voice report slot %d %s: active %d, live %d, human %d, hears %d (engine listening %d), datagram %d/%d\n",
				i + 1, ent ? STRING(ent->v.netname) : "?", cl->IsActive() ? 1 : 0, CBotIsLivePlayer(ent) ? 1 : 0,
				ent && BrainFeed::IsHuman(ent) ? 1 : 0, ent && CanHear(ent, sender) ? 1 : 0,
				g_engfuncs.pfnVoice_GetClientListening(i + 1, sender + 1),
				dg ? dg->cursize : -1, dg ? dg->maxsize : -1);
			SERVER_PRINT(line);
		}
	}

	std::deque<std::string> Encode(std::vector<short> &pcm, int rate)
	{
		std::deque<std::string> frames;
		int err = 0;
		OpusEncoder *enc = opus_encoder_create(rate, 1, OPUS_APPLICATION_VOIP, &err);
		if (!enc)
			return frames;
		opus_encoder_ctl(enc, OPUS_SET_BITRATE(32000));
		opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
		int frame = rate / 50;
		pcm.resize((pcm.size() + frame - 1) / frame * frame, 0);
		for (size_t at = 0; at < pcm.size(); at += frame)
		{
			unsigned char out[1275];
			int n = opus_encode(enc, pcm.data() + at, frame, out, sizeof(out));
			if (n > 0)
				frames.emplace_back(reinterpret_cast<const char *>(out), n);
		}
		opus_encoder_destroy(enc);
		return frames;
	}

	void Queue(int index, int userid, int rate, const std::deque<std::string> &frames, bool loopback)
	{
		Talk &t = g_talk[index];
		if (t.userid != userid || t.frames.empty())
		{
			t = Talk();
			t.userid = userid;
			t.rate = rate;
		}
		t.loopback = t.loopback || loopback;
		t.frames.insert(t.frames.end(), frames.begin(), frames.end());
	}

	int SendRate()
	{
		int rate = BrainConfig::Get().voiceRate;
		if (rate != 8000 && rate != 12000 && rate != 16000 && rate != 24000 && rate != 48000)
			rate = 24000;
		return rate;
	}

	int g_echo = -1;
	unsigned int g_echoPackets = 0;

	void Pace(int index)
	{
		Talk &t = g_talk[index];
		if (t.frames.empty())
			return;
		if (!PlayerAt(index, t.userid))
		{
			t = Talk();
			return;
		}
		float now = gpGlobals->time;
		if (t.start < 0.f)
			t.start = now;
		int due = static_cast<int>((now - t.start) * 50.f) + kLookahead;
		while (t.sent < due && !t.frames.empty())
		{
			std::string packet = Packet(t, true);
			int heard = Send(index, packet);
			if (heard > t.receivers)
				t.receivers = heard;
			if (t.loopback)
				Receive(index, reinterpret_cast<const unsigned char *>(packet.data()), static_cast<int>(packet.size()));
		}
		if (t.frames.empty())
		{
			char line[160];
			snprintf(line, sizeof(line), "CrossBot: voice sent %d frame(s) from slot %d to %d listener(s)\n", t.sent, index + 1,
				t.receivers);
			SERVER_PRINT(line);
			t = Talk();
		}
	}

	void OnNetCommand(IRehldsHook_HandleNetCommand *chain, IGameClient *cl, int8 opcode)
	{
		if (opcode == kClcVoiceData && cl)
		{
			int *readCount = g_RehldsFuncs->GetMsgReadCount();
			int *badRead = g_RehldsFuncs->GetMsgBadRead();
			int saveCount = *readCount;
			int saveBad = *badRead;
			int len = g_RehldsFuncs->MSG_ReadShort();
			unsigned char buf[4096];
			int index = cl->GetId();
			if (len > 0 && len <= static_cast<int>(sizeof(buf)) && g_RehldsFuncs->MSG_ReadBuf(len, buf) != -1 && !*badRead &&
				index >= 0 && index < kMaxPlayers)
			{
				edict_t *ent = cl->GetEdict();
				if (g_echo >= 0 && g_echo != index && CBotIsLivePlayer(ent) && BrainFeed::IsHuman(ent))
				{
					if (!g_echoPackets++)
					{
						char line[128];
						snprintf(line, sizeof(line), "CrossBot: voice echo from slot %d, %d bytes, op %d at byte 8\n", index + 1, len,
							len > 8 ? buf[8] : -1);
						SERVER_PRINT(line);
						Report(g_echo);
					}
					int heard = Send(g_echo, std::string(reinterpret_cast<const char *>(buf), len));
					if (heard == 0 && (g_echoPackets & 63) == 1)
						SERVER_PRINT("CrossBot: voice echo reached nobody\n");
				}
				const BrainConfig::Config &config = BrainConfig::Get();
				if (config.enabled && config.voice && CBotIsLivePlayer(ent) && BrainFeed::IsHuman(ent) &&
					Brain::Hears(g_engfuncs.pfnGetPlayerUserId(ent)))
					Receive(index, buf, len);
			}
			*readCount = saveCount;
			*badRead = saveBad;
		}
		chain->callNext(cl, opcode);
	}
}

namespace BrainVoice
{

void Attach()
{
	if (g_hooked)
		return;
	if (!meta_init_rehlds_api())
	{
		SERVER_PRINT("CrossBot: voice off: the engine is not ReHLDS\n");
		return;
	}
	g_RehldsHookchains->HandleNetCommand()->registerHook(&OnNetCommand);
	g_hooked = true;
}

void Detach()
{
	if (g_hooked)
		g_RehldsHookchains->HandleNetCommand()->unregisterHook(&OnNetCommand);
	g_hooked = false;
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_quit = true;
		g_work.clear();
	}
	g_wake.notify_all();
	for (std::thread &t : g_threads)
		t.join();
	g_threads.clear();
	g_done.clear();
	for (Listener &l : g_listen)
		ResetListener(l);
}

void Frame()
{
	std::deque<std::function<void()>> done;
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		done.swap(g_done);
	}
	for (std::function<void()> &fn : done)
		fn();

	const BrainConfig::Config &config = BrainConfig::Get();
	for (int i = 0; i < kMaxPlayers; ++i)
	{
		Listener &l = g_listen[i];
		if (!l.pcm.empty() && gpGlobals->time - l.last > config.voiceSilence)
			Finish(i);
		Pace(i);
	}
}

void OnMapEnd()
{
	for (Listener &l : g_listen)
		ResetListener(l);
	for (Talk &t : g_talk)
		t = Talk();
	g_echo = -1;
}

bool Speak(CBot &bot, const char *text, bool loopback, std::string &result)
{
	const BrainConfig::Config &config = BrainConfig::Get();
	if (!config.enabled || !config.voice || (!g_hooked && !loopback))
	{
		result = "error: voice is off";
		return false;
	}
	std::string clean = BrainTools::Sanitize(text, kMaxSpeech);
	if (clean.empty())
	{
		result = "error: nothing to say";
		return false;
	}
	int index = g_engfuncs.pfnIndexOfEdict(bot.pEdict) - 1;
	if (index < 0 || index >= kMaxPlayers)
	{
		result = "error: bad bot slot";
		return false;
	}
	int userid = g_engfuncs.pfnGetPlayerUserId(bot.pEdict);
	if (!loopback)
		BrainFeed::NoteChat(bot.pEdict, TeamOnly(index), clean.c_str(), BrainFeed::EV_VOICE);

	int rate = SendRate();
	std::string body = "{\"model\":" + Json::Quote(config.ttsModel.c_str()) + ",\"input\":" + Json::Quote(clean.c_str()) +
		",\"response_format\":\"pcm\"";
	if (!config.ttsVoice.empty())
		body += ",\"voice\":" + Json::Quote(config.ttsVoice.c_str());
	body += "}";
	BrainConfig::Config cfg = config;
	Async([=] {
		BrainHttp::Response r;
		bool ok = BrainHttp::Post(cfg.ttsUrl, cfg.key, body, cfg.ttsTimeout, r) && r.body.size() >= 2;
		int from = TypeParam(r.type, "rate=", 44100);
		int channels = TypeParam(r.type, "channels=", 1);
		std::vector<short> pcm;
		if (ok)
			pcm = Resample(reinterpret_cast<const short *>(r.body.data()), r.body.size() / 2, channels, from, rate);

		std::deque<std::string> frames;
		if (ok)
			frames = Encode(pcm, rate);

		char head[256];
		snprintf(head, sizeof(head),
			"{\"time\":%lld,\"kind\":\"tts\",\"status\":%d,\"seconds\":%.3f,\"bytes\":%zu,\"frames\":%zu,\"type\":",
			static_cast<long long>(time(nullptr)), r.status, r.seconds, r.body.size(), frames.size());
		std::string trace = std::string(head) + Json::Quote(r.type.c_str()) + ",\"error\":" + Json::Quote(r.error.c_str()) +
			",\"request\":" + body + (ok ? std::string() : ",\"response\":" + Json::Quote(r.body.substr(0, 1000).c_str())) + "}";
		Done([=] {
			Trace(trace);
			++g_ttsCalls;
			g_ttsSeconds = r.seconds;
			if (frames.empty())
			{
				g_lastError = "tts http " + std::to_string(r.status) + " " + r.error + " " + r.body.substr(0, 160);
				std::string line = "CrossBot: voice tts failed: " + g_lastError + "\n";
				SERVER_PRINT(line.c_str());
				return;
			}
			if (!PlayerAt(index, userid))
				return;
			Queue(index, userid, rate, frames, loopback);
			char line[160];
			snprintf(line, sizeof(line), "CrossBot: voice tts %.2fs, %.1fs of speech from slot %d\n", r.seconds,
				frames.size() / 50.f, index + 1);
			SERVER_PRINT(line);
		});
	});
	result = "ok: speaking";
	return true;
}

void Tone(CBot &bot, float seconds, float hz)
{
	int index = g_engfuncs.pfnIndexOfEdict(bot.pEdict) - 1;
	if (index < 0 || index >= kMaxPlayers)
		return;
	int rate = SendRate();
	std::vector<short> pcm(static_cast<size_t>(seconds * rate));
	for (size_t i = 0; i < pcm.size(); ++i)
		pcm[i] = static_cast<short>(8000.f * sinf(6.2831853f * hz * i / rate));
	std::deque<std::string> frames = Encode(pcm, rate);
	Queue(index, g_engfuncs.pfnGetPlayerUserId(bot.pEdict), rate, frames, false);
	char line[160];
	snprintf(line, sizeof(line), "CrossBot: voice tone %.0f Hz, %.1fs, %d Hz opus, %zu frame(s) from slot %d\n", hz, seconds,
		rate, frames.size(), index + 1);
	SERVER_PRINT(line);
	Report(index);
}

void Echo(CBot *bot)
{
	g_echo = bot ? g_engfuncs.pfnIndexOfEdict(bot->pEdict) - 1 : -1;
	g_echoPackets = 0;
}

void PrintStatus()
{
	const BrainConfig::Config &config = BrainConfig::Get();
	char line[256];
	snprintf(line, sizeof(line), "CrossBot: voice %s%s, stt %u (last %.2fs), tts %u (last %.2fs)%s%s\n",
		config.voice ? "on" : "off", g_hooked ? "" : " (no ReHLDS hook, cannot hear or speak)", g_sttCalls, g_sttSeconds,
		g_ttsCalls, g_ttsSeconds, g_lastError.empty() ? "" : ", last error: ", g_lastError.substr(0, 120).c_str());
	SERVER_PRINT(line);
	for (int i = 0; i < kMaxPlayers; ++i)
	{
		const Listener &l = g_listen[i];
		if (!l.packets)
			continue;
		edict_t *ent = PlayerAt(i, l.userid);
		snprintf(line, sizeof(line), "CrossBot:   heard %s: %s, %d Hz, %u packet(s), %u rejected, %u utterance(s)\n",
			ent ? CBotPlayerNameOf(ent) : "?", l.codec, l.rate, l.packets, l.rejected, l.utterances);
		SERVER_PRINT(line);
	}
}

}
