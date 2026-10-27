#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <deque>
#include <mutex>
#include <thread>

#include "brain_http.h"
#include "brain_worker.h"
#include "json.h"

namespace
{
	std::mutex g_mutex;
	std::condition_variable g_wake;
	std::deque<BrainWorker::Job> g_jobs;
	std::deque<BrainWorker::Result> g_results;
	std::thread g_thread;
	bool g_quit = false;
	std::atomic<int> g_busy(0);

	std::string ApiError(const BrainHttp::Response &r)
	{
		Json::Doc doc;
		std::string msg;
		if (doc.Parse(r.body))
			msg = doc.GetStr(doc.Member(doc.Root(), "error"), "message");
		char buf[160];
		snprintf(buf, sizeof(buf), "http %d%s%s%s", r.status, r.error.empty() ? "" : " ", r.error.c_str(),
			msg.empty() ? "" : ": ");
		return buf + msg.substr(0, 200);
	}

	std::string Embed(const std::string &text)
	{
		Json::Doc doc;
		if (!doc.Parse(text))
			return Json::Quote(text.c_str());
		std::string flat;
		flat.reserve(text.size());
		for (char c : text)
		{
			if (c != '\n' && c != '\r')
				flat += c;
		}
		size_t start = flat.find_first_not_of(" \t");
		return start == std::string::npos ? std::string("null") : flat.substr(start);
	}

	void Trace(const BrainWorker::Job &job, const char *kind, const std::string &url, const std::string &request,
		const BrainHttp::Response &r)
	{
		if (job.tracePath.empty())
			return;
		FILE *f = fopen(job.tracePath.c_str(), "ab");
		if (!f)
			return;
		char head[160];
		snprintf(head, sizeof(head), "{\"time\":%lld,\"job\":%u,\"brain\":%d,\"kind\":\"%s\",\"status\":%d,\"seconds\":%.3f,",
			static_cast<long long>(time(nullptr)), job.id, job.brain, kind, r.status, r.seconds);
		std::string line = std::string(head) + "\"url\":" + Json::Quote(url.c_str()) + ",\"error\":" +
			Json::Quote(r.error.c_str()) + ",\"request\":" + Embed(request) + ",\"response\":" + Embed(r.body) + "}\n";
		fwrite(line.data(), 1, line.size(), f);
		fclose(f);
	}

	float Answer(const Json::Doc &doc, int answers, const char *id, const char *field)
	{
		return doc.GetNum(doc.Member(answers, id), field, 0.f);
	}

	float LikelyLevel(const Json::Doc &doc, int answers, const char *id)
	{
		int answer = doc.Member(answers, id);
		int probs = doc.Member(answer, "probabilities");
		if (doc.TypeOf(probs) != Json::T_OBJECT)
			return doc.GetNum(answer, "score", 0.f);
		float best = -1.f;
		float level = 0.f;
		for (int i = 0; i < doc.Count(probs); ++i)
		{
			int key = doc.Key(probs, i);
			float p = 0.f;
			float k = 0.f;
			if (!doc.Num(key + 1, p) || !doc.Num(key, k))
				continue;
			if (p > best)
			{
				best = p;
				level = k;
			}
		}
		return level;
	}

	void RunJev(const BrainWorker::Job &job, BrainWorker::Result &res)
	{
		BrainHttp::Response r;
		bool ok = BrainHttp::Post(job.config.jevUrl, job.config.key, job.jevBody, job.config.jevTimeout, r);
		Trace(job, "jev", job.config.jevUrl, job.jevBody, r);
		res.jevSeconds = r.seconds;
		res.jevRaw = r.body.substr(0, 4000);
		if (!ok)
		{
			res.error = "jev " + ApiError(r);
			return;
		}

		Json::Doc doc;
		if (!doc.Parse(r.body))
		{
			res.error = "jev answered something that is not JSON";
			return;
		}
		int root = doc.Root();
		int answers = doc.Member(root, "answers");
		if (doc.TypeOf(answers) != Json::T_OBJECT)
		{
			res.error = "jev answer has no 'answers'";
			return;
		}

		res.jevOk = true;
		res.act = Answer(doc, answers, "act", "noul");
		res.remember = Answer(doc, answers, "remember", "noul");
		res.ignoreSeconds = BrainConfig::LevelSeconds(job.config.ignoreLevels, LikelyLevel(doc, answers, "ignore"));
		res.recheckSeconds = BrainConfig::LevelSeconds(job.config.recheckLevels, LikelyLevel(doc, answers, "recheck"));
		res.involve.resize(job.bots.size());
		for (size_t i = 0; i < job.bots.size(); ++i)
		{
			char id[24];
			snprintf(id, sizeof(id), "involve_%zu", i);
			res.involve[i] = Answer(doc, answers, id, "noul");
		}
		res.jevTokens = static_cast<int>(doc.GetNum(doc.Member(root, "usage"), "input_tokens", 0.f));
	}

	std::string Verdict(const BrainWorker::Job &job, const BrainWorker::Result &res)
	{
		std::string v = "\n## Gate\n";
		char buf[160];
		snprintf(buf, sizeof(buf), "Someone should respond: p=%.2f. Worth remembering: p=%.2f.\n", res.act, res.remember);
		v += buf;
		std::string involved;
		for (size_t i = 0; i < job.bots.size(); ++i)
		{
			if (res.involve[i] < job.config.involve)
				continue;
			snprintf(buf, sizeof(buf), "%s%s (%.2f)", involved.empty() ? "" : ", ", job.bots[i].c_str(), res.involve[i]);
			involved += buf;
		}
		v += "Likely responders: " + (involved.empty() ? std::string("none picked") : involved) + ".\n";
		v += job.config.gate;
		return v;
	}

	void RunLlm(const BrainWorker::Job &job, BrainWorker::Result &res)
	{
		res.llmCalled = true;
		std::string body = "{\"model\":" + Json::Quote(job.config.llmModel.c_str()) +
			",\"messages\":[{\"role\":\"system\",\"content\":" + Json::Quote(job.config.system.c_str()) +
			"},{\"role\":\"user\",\"content\":" + Json::Quote((job.context + Verdict(job, res)).c_str()) + "}]" +
			",\"tools\":" + job.config.tools + ",\"tool_choice\":\"auto\"";
		char buf[64];
		snprintf(buf, sizeof(buf), ",\"max_tokens\":%d", job.config.llmMaxTokens);
		body += buf;
		if (job.config.reasoning == "off" || job.config.reasoning == "none")
			body += ",\"reasoning\":{\"enabled\":false}";
		else if (!job.config.reasoning.empty())
			body += ",\"reasoning\":{\"effort\":" + Json::Quote(job.config.reasoning.c_str()) + "}";
		body += "}";

		BrainHttp::Response r;
		bool ok = BrainHttp::Post(job.config.llmUrl, job.config.key, body, job.config.llmTimeout, r);
		Trace(job, "llm", job.config.llmUrl, body, r);
		res.llmSeconds = r.seconds;
		res.llmRaw = r.body.substr(0, 6000);
		if (!ok)
		{
			res.error = "llm " + ApiError(r);
			return;
		}

		Json::Doc doc;
		if (!doc.Parse(r.body))
		{
			res.error = "llm answered something that is not JSON";
			return;
		}
		int root = doc.Root();
		int usage = doc.Member(root, "usage");
		res.llmIn = static_cast<int>(doc.GetNum(usage, "prompt_tokens", 0.f));
		res.llmOut = static_cast<int>(doc.GetNum(usage, "completion_tokens", 0.f));

		int choice = doc.Element(doc.Member(root, "choices"), 0);
		int message = doc.Member(choice, "message");
		if (doc.TypeOf(message) != Json::T_OBJECT)
		{
			res.error = "llm answer has no message";
			return;
		}
		res.llmOk = true;
		res.content = doc.GetStr(message, "content");
		int calls = doc.Member(message, "tool_calls");
		for (int i = 0; i < doc.Count(calls); ++i)
		{
			int fn = doc.Member(doc.Element(calls, i), "function");
			BrainWorker::Call call;
			call.name = doc.GetStr(fn, "name");
			call.args = doc.GetStr(fn, "arguments", "{}");
			if (!call.name.empty())
				res.calls.push_back(call);
		}
		if (res.calls.empty() && doc.GetStr(choice, "finish_reason") == "length")
			res.error = "llm ran out of tokens before answering (raise max_tokens or set reasoning off)";
	}

	void Process(const BrainWorker::Job &job, BrainWorker::Result &res)
	{
		res.brain = job.brain;
		res.id = job.id;
		res.epoch = job.epoch;

		RunJev(job, res);
		if (!res.jevOk)
			return;
		if (res.ignoreSeconds >= job.config.ignoreMin)
			return;
		if (res.act < job.config.act && res.remember < job.config.remember)
			return;
		RunLlm(job, res);
	}

	void Loop()
	{
		for (;;)
		{
			BrainWorker::Job job;
			{
				std::unique_lock<std::mutex> lock(g_mutex);
				g_wake.wait(lock, [] { return g_quit || !g_jobs.empty(); });
				if (g_quit)
					return;
				job = std::move(g_jobs.front());
				g_jobs.pop_front();
			}

			BrainWorker::Result res;
			Process(job, res);

			std::lock_guard<std::mutex> lock(g_mutex);
			g_results.push_back(std::move(res));
			--g_busy;
		}
	}
}

namespace BrainWorker
{

void Submit(Job &&job)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	if (!g_thread.joinable())
	{
		g_quit = false;
		g_thread = std::thread(Loop);
	}
	++g_busy;
	g_jobs.push_back(std::move(job));
	g_wake.notify_one();
}

bool Poll(Result &out)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	if (g_results.empty())
		return false;
	out = std::move(g_results.front());
	g_results.pop_front();
	return true;
}

int Busy()
{
	return g_busy;
}

void Stop()
{
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		g_quit = true;
		g_jobs.clear();
	}
	g_wake.notify_all();
	if (g_thread.joinable())
		g_thread.join();
	g_results.clear();
	g_busy = 0;
	BrainHttp::Shutdown();
}

}
