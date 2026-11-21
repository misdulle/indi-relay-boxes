#pragma once
// KMTronic WEB relay commands with automatic fallback.
//   FF style : GET /FF0N01 (on) / GET /FF0N00 (off)
//   CGI style: GET /relays.cgi?relay=N toggles; we read /status.xml first and toggle only if needed.
// AUTO tries FF and falls back to CGI (permanently, via `style`) when the box answers 404.
#include "http_client.h"
#include <regex>
#include <cstdio>

namespace kmt
{
enum Style { AUTO = 0, FF = 1, CGI = 2 };

inline bool readRelay(const std::string &host, int port, const std::string &user, const std::string &pass,
                      int timeoutMs, int relay, int &state, std::string &err)
{
    int status = 0;
    std::string body;
    if (!httpc::get(host, port, "/status.xml", timeoutMs, status, body, err, httpc::basicAuthHeader(user, pass)))
        return false;
    if (status != 200)
    {
        err = "HTTP status " + std::to_string(status) + " for /status.xml";
        return false;
    }
    std::regex re("<relay" + std::to_string(relay) + ">\\s*([01])\\s*</relay");
    std::smatch m;
    if (!std::regex_search(body, m, re))
    {
        err = "status.xml: relay " + std::to_string(relay) + " not found";
        return false;
    }
    state = (m[1].str() == "1") ? 1 : 0;
    return true;
}

// `usedPath` receives the URL path that finally worked (or failed), for logging.
inline bool setRelay(const std::string &host, int port, const std::string &user, const std::string &pass,
                     int timeoutMs, int relay, bool on, Style &style, std::string &err, std::string *usedPath = nullptr)
{
    const std::string hdr = httpc::basicAuthHeader(user, pass);
    int status = 0;
    std::string body;
    char path[48];

    if (style != CGI)
    {
        snprintf(path, sizeof(path), "/FF%02d%02d", relay, on ? 1 : 0);
        if (usedPath) *usedPath = path;
        if (!httpc::get(host, port, path, timeoutMs, status, body, err, hdr))
            return false;
        if (status == 200)
            return true;
        if (status == 404 && style == AUTO)
            style = CGI; // fall through to the toggle command
        else
        {
            err = "HTTP status " + std::to_string(status) + " for " + path;
            return false;
        }
    }

    int cur = -1;
    if (!readRelay(host, port, user, pass, timeoutMs, relay, cur, err))
        return false;
    if ((cur == 1) == on)
        return true;
    snprintf(path, sizeof(path), "/relays.cgi?relay=%d", relay);
    if (usedPath) *usedPath = path;
    if (!httpc::get(host, port, path, timeoutMs, status, body, err, hdr))
        return false;
    if (status != 200)
    {
        err = "HTTP status " + std::to_string(status) + " for " + path;
        return false;
    }
    return true;
}
} // namespace kmt
