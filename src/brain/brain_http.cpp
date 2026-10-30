#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>

#include "brain_http.h"

#if defined(_WIN32)

#include <windows.h>
#include <winhttp.h>

namespace
{
	HINTERNET g_session = nullptr;
	std::mutex g_sessionMutex;

	std::wstring Wide(const std::string &s)
	{
		if (s.empty())
			return std::wstring();
		int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
		std::wstring w(static_cast<size_t>(n), L'\0');
		MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), &w[0], n);
		return w;
	}

	std::string LastError(const char *what)
	{
		char buf[96];
		snprintf(buf, sizeof(buf), "%s failed (winhttp error %lu)", what, GetLastError());
		return buf;
	}

	struct Handle
	{
		HINTERNET h = nullptr;
		~Handle()
		{
			if (h)
				WinHttpCloseHandle(h);
		}
	};
}

namespace BrainHttp
{

bool Post(const std::string &url, const std::string &key, const std::string &body, float timeout, Response &out)
{
	auto start = std::chrono::steady_clock::now();
	out = Response();

	std::wstring wurl = Wide(url);
	URL_COMPONENTS uc = {};
	uc.dwStructSize = sizeof(uc);
	uc.dwHostNameLength = static_cast<DWORD>(-1);
	uc.dwUrlPathLength = static_cast<DWORD>(-1);
	uc.dwExtraInfoLength = static_cast<DWORD>(-1);
	if (!WinHttpCrackUrl(wurl.c_str(), 0, 0, &uc))
	{
		out.error = "bad url";
		return false;
	}
	std::wstring host(uc.lpszHostName, uc.dwHostNameLength);
	std::wstring path(uc.lpszUrlPath, uc.dwUrlPathLength);
	if (uc.lpszExtraInfo && uc.dwExtraInfoLength)
		path.append(uc.lpszExtraInfo, uc.dwExtraInfoLength);
	bool secure = uc.nScheme == INTERNET_SCHEME_HTTPS;

	HINTERNET session;
	{
		std::lock_guard<std::mutex> lock(g_sessionMutex);
		if (!g_session)
			g_session = WinHttpOpen(L"CrossBot/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
				WINHTTP_NO_PROXY_BYPASS, 0);
		session = g_session;
	}
	if (!session)
	{
		out.error = LastError("WinHttpOpen");
		return false;
	}

	int ms = static_cast<int>(timeout * 1000.f);
	if (ms < 1000)
		ms = 1000;

	Handle connect;
	connect.h = WinHttpConnect(session, host.c_str(), uc.nPort, 0);
	if (!connect.h)
	{
		out.error = LastError("WinHttpConnect");
		return false;
	}

	Handle request;
	request.h = WinHttpOpenRequest(connect.h, L"POST", path.c_str(), nullptr, WINHTTP_NO_REFERER,
		WINHTTP_DEFAULT_ACCEPT_TYPES, secure ? WINHTTP_FLAG_SECURE : 0);
	if (!request.h)
	{
		out.error = LastError("WinHttpOpenRequest");
		return false;
	}
	WinHttpSetTimeouts(request.h, ms, ms, ms, ms);

	std::wstring headers = L"Content-Type: application/json\r\nX-Title: CrossBot\r\n";
	if (!key.empty())
		headers += L"Authorization: Bearer " + Wide(key) + L"\r\n";

	if (!WinHttpSendRequest(request.h, headers.c_str(), static_cast<DWORD>(-1),
			const_cast<char *>(body.data()), static_cast<DWORD>(body.size()), static_cast<DWORD>(body.size()), 0) ||
		!WinHttpReceiveResponse(request.h, nullptr))
	{
		out.error = LastError("request");
		out.seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
		return false;
	}

	DWORD status = 0;
	DWORD size = sizeof(status);
	WinHttpQueryHeaders(request.h, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
		&status, &size, WINHTTP_NO_HEADER_INDEX);
	out.status = static_cast<int>(status);

	wchar_t type[128] = {};
	DWORD typeSize = sizeof(type);
	if (WinHttpQueryHeaders(request.h, WINHTTP_QUERY_CONTENT_TYPE, WINHTTP_HEADER_NAME_BY_INDEX, type, &typeSize,
			WINHTTP_NO_HEADER_INDEX))
	{
		for (const wchar_t *c = type; *c; ++c)
			out.type += static_cast<char>(*c);
	}

	for (;;)
	{
		DWORD avail = 0;
		if (!WinHttpQueryDataAvailable(request.h, &avail))
		{
			out.error = LastError("read");
			break;
		}
		if (avail == 0)
			break;
		size_t at = out.body.size();
		out.body.resize(at + avail);
		DWORD got = 0;
		if (!WinHttpReadData(request.h, &out.body[at], avail, &got))
		{
			out.error = LastError("read");
			out.body.resize(at);
			break;
		}
		out.body.resize(at + got);
		if (std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count() > timeout)
		{
			out.error = "timed out";
			break;
		}
	}

	out.seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
	return out.error.empty() && out.status >= 200 && out.status < 300;
}

void Shutdown()
{
	std::lock_guard<std::mutex> lock(g_sessionMutex);
	if (g_session)
	{
		WinHttpCloseHandle(g_session);
		g_session = nullptr;
	}
}

}

#else

namespace BrainHttp
{

bool Post(const std::string &, const std::string &, const std::string &, float, Response &out)
{
	out = Response();
	out.error = "no HTTP client on this platform";
	return false;
}

void Shutdown()
{
}

}

#endif
