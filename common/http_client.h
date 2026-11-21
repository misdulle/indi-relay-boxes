#pragma once
// Tiny blocking HTTP/1.x GET client (IPv4, no TLS, no dependencies).
// Good enough for relay boxes that answer "HTTP/1.0 200 OK" and close the connection.
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>

namespace httpc
{
// "Authorization: Basic ..." header line for user:pass (empty string if user is empty).
inline std::string basicAuthHeader(const std::string &user, const std::string &pass)
{
    if (user.empty())
        return std::string();
    static const char *tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string in = user + ":" + pass, out;
    size_t i = 0;
    for (; i + 2 < in.size(); i += 3)
    {
        unsigned v = (static_cast<unsigned char>(in[i]) << 16) | (static_cast<unsigned char>(in[i + 1]) << 8) |
                     static_cast<unsigned char>(in[i + 2]);
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += tbl[v & 63];
    }
    if (i + 1 == in.size())
    {
        unsigned v = static_cast<unsigned char>(in[i]) << 16;
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += "==";
    }
    else if (i + 2 == in.size())
    {
        unsigned v = (static_cast<unsigned char>(in[i]) << 16) | (static_cast<unsigned char>(in[i + 1]) << 8);
        out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += "=";
    }
    return "Authorization: Basic " + out + "\r\n";
}

inline long long nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// extraHeaders: complete header lines ending in "\r\n" (e.g. "Authorization: Basic ...\r\n").
// On success returns true with the HTTP status in `status` and the body in `body`.
inline bool get(const std::string &host, int port, const std::string &path, int timeoutMs,
                int &status, std::string &body, std::string &err, const std::string &extraHeaders = "")
{
    status = 0;
    body.clear();
    const long long deadline = nowMs() + timeoutMs;

    addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo *res = nullptr;
    int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
    if (rc != 0 || res == nullptr)
    {
        err = std::string("resolve ") + host + ": " + gai_strerror(rc);
        return false;
    }

    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, res->ai_protocol);
    if (fd < 0) { err = strerror(errno); freeaddrinfo(res); return false; }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    auto remaining = [&]() -> int
    {
        long long r = deadline - nowMs();
        return r < 0 ? 0 : static_cast<int>(r);
    };

    rc = connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (rc < 0 && errno != EINPROGRESS)
    {
        err = std::string("connect: ") + strerror(errno);
        close(fd);
        return false;
    }
    if (rc < 0)
    {
        pollfd p{fd, POLLOUT, 0};
        if (poll(&p, 1, remaining()) <= 0)
        {
            err = "connect timeout";
            close(fd);
            return false;
        }
        int soerr = 0;
        socklen_t sl = sizeof(soerr);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &sl);
        if (soerr != 0)
        {
            err = std::string("connect: ") + strerror(soerr);
            close(fd);
            return false;
        }
    }

    std::string req = "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\n" + extraHeaders +
                      "Connection: close\r\n\r\n";
    size_t sent = 0;
    while (sent < req.size())
    {
        pollfd p{fd, POLLOUT, 0};
        if (poll(&p, 1, remaining()) <= 0) { err = "send timeout"; close(fd); return false; }
        ssize_t n = send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
        if (n < 0 && errno != EAGAIN && errno != EINTR) { err = std::string("send: ") + strerror(errno); close(fd); return false; }
        if (n > 0) sent += static_cast<size_t>(n);
    }

    std::string raw;
    char buf[4096];
    for (;;)
    {
        pollfd p{fd, POLLIN, 0};
        int pr = poll(&p, 1, remaining());
        if (pr == 0) { err = "read timeout"; close(fd); return false; }
        if (pr < 0 && errno == EINTR) continue;
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n > 0) { raw.append(buf, static_cast<size_t>(n)); if (raw.size() > (1u << 20)) break; continue; }
        if (n == 0) break; // server closed: end of response
        if (errno == EAGAIN || errno == EINTR) continue;
        err = std::string("recv: ") + strerror(errno);
        close(fd);
        return false;
    }
    close(fd);

    if (raw.compare(0, 5, "HTTP/") != 0)
    {
        err = "not an HTTP response";
        return false;
    }
    size_t sp = raw.find(' ');
    status = (sp == std::string::npos) ? 0 : atoi(raw.c_str() + sp + 1);
    size_t hdrEnd = raw.find("\r\n\r\n");
    body = (hdrEnd == std::string::npos) ? std::string() : raw.substr(hdrEnd + 4);
    return true;
}
} // namespace httpc
