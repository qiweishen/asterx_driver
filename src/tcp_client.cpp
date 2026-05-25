// SPDX-License-Identifier: BSD-3-Clause
#include "tcp_client.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>

namespace asterx {

namespace {

inline int posix_error() noexcept { return errno; }

std::string errno_string(int e) {
    char buf[256] = {0};
    // Use the XSI-compliant strerror_r (returns int) — glibc has both, so we
    // pick portably via the GNU variant if available; here use the safe form.
#if defined(_GNU_SOURCE)
    return std::string(strerror_r(e, buf, sizeof(buf)));
#else
    if (strerror_r(e, buf, sizeof(buf)) != 0) {
        std::snprintf(buf, sizeof(buf), "errno=%d", e);
    }
    return std::string(buf);
#endif
}

void set_nonblocking(int fd, bool on) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        throw TcpError("fcntl(F_GETFL) failed: " + errno_string(posix_error()));
    }
    flags = on ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    if (::fcntl(fd, F_SETFL, flags) < 0) {
        throw TcpError("fcntl(F_SETFL) failed: " + errno_string(posix_error()));
    }
}

}  // namespace

TcpClient::~TcpClient() { close(); }

void TcpClient::close() noexcept {
    if (fd_ >= 0) {
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
}

void TcpClient::connect(const std::string& host,
                        std::uint16_t      port,
                        std::chrono::milliseconds connect_timeout) {
    if (fd_ >= 0) {
        throw TcpError("TcpClient::connect: already open");
    }

    addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* res = nullptr;
    const std::string port_str = std::to_string(port);
    int gai = ::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res);
    if (gai != 0) {
        throw TcpError("getaddrinfo(" + host + ":" + port_str + ") failed: " +
                       gai_strerror(gai));
    }

    int last_err = 0;
    for (addrinfo* p = res; p != nullptr; p = p->ai_next) {
        int s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (s < 0) { last_err = posix_error(); continue; }

        // Disable Nagle so command replies come back immediately.
        int yes = 1;
        ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        set_nonblocking(s, true);

        int rc = ::connect(s, p->ai_addr, p->ai_addrlen);
        if (rc == 0) {
            set_nonblocking(s, false);
            fd_ = s;
            ::freeaddrinfo(res);
            return;
        }
        if (errno != EINPROGRESS && errno != EWOULDBLOCK) {
            last_err = posix_error();
            ::close(s);
            continue;
        }

        // Wait for connect to finish or time out.
        pollfd pfd{};
        pfd.fd     = s;
        pfd.events = POLLOUT;
        int pr = ::poll(&pfd, 1, static_cast<int>(connect_timeout.count()));
        if (pr <= 0) {
            last_err = (pr == 0) ? ETIMEDOUT : posix_error();
            ::close(s);
            continue;
        }
        int so_err = 0;
        socklen_t sl = sizeof(so_err);
        if (::getsockopt(s, SOL_SOCKET, SO_ERROR, &so_err, &sl) < 0 || so_err != 0) {
            last_err = so_err ? so_err : posix_error();
            ::close(s);
            continue;
        }
        set_nonblocking(s, false);
        fd_ = s;
        ::freeaddrinfo(res);
        return;
    }

    ::freeaddrinfo(res);
    throw TcpError("connect(" + host + ":" + port_str + ") failed: " +
                   errno_string(last_err));
}

void TcpClient::write_all(ConstByteSpan buf) {
    if (fd_ < 0) throw TcpError("write_all: socket not open");
    const std::uint8_t* p   = buf.data();
    std::size_t         rem = buf.size();
    while (rem > 0) {
        ssize_t n = ::send(fd_, p, rem, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            throw TcpError("send() failed: " + errno_string(posix_error()));
        }
        if (n == 0) {
            throw TcpError("send() returned 0 (peer closed)");
        }
        p   += n;
        rem -= static_cast<std::size_t>(n);
    }
}

void TcpClient::write_all(const std::string& s) {
    write_all(ConstByteSpan{reinterpret_cast<const std::uint8_t*>(s.data()),
                            s.size()});
}

std::size_t TcpClient::read_some(MutByteSpan buf,
                                 std::chrono::milliseconds io_timeout) {
    if (fd_ < 0) throw TcpError("read_some: socket not open");

    pollfd pfd{};
    pfd.fd     = fd_;
    pfd.events = POLLIN;
    int pr = ::poll(&pfd, 1, static_cast<int>(io_timeout.count()));
    if (pr < 0) {
        if (errno == EINTR) return 0;
        throw TcpError("poll() failed: " + errno_string(posix_error()));
    }
    if (pr == 0) {
        throw TcpError("read timeout (" + std::to_string(io_timeout.count()) + " ms)");
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        // Drain anything pending; recv() will tell us the real reason.
    }
    ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
    if (n < 0) {
        if (errno == EINTR) return 0;
        throw TcpError("recv() failed: " + errno_string(posix_error()));
    }
    return static_cast<std::size_t>(n);  // 0 = clean close
}

void TcpClient::read_exact(MutByteSpan buf, std::chrono::milliseconds io_timeout) {
    std::uint8_t* p   = buf.data();
    std::size_t   rem = buf.size();
    const auto deadline = std::chrono::steady_clock::now() + io_timeout;
    while (rem > 0) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            throw TcpError("read_exact: timeout");
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        std::size_t got = read_some(MutByteSpan{p, rem}, remaining);
        if (got == 0) throw TcpError("read_exact: peer closed");
        p   += got;
        rem -= got;
    }
}

std::string TcpClient::read_until(const std::string& terminator,
                                  std::chrono::milliseconds total_timeout,
                                  std::size_t               max_bytes) {
    std::string acc;
    acc.reserve(256);
    const auto deadline = std::chrono::steady_clock::now() + total_timeout;
    std::uint8_t scratch[512];

    while (true) {
        if (acc.find(terminator) != std::string::npos) return acc;
        if (acc.size() >= max_bytes) {
            throw TcpError("read_until: max_bytes (" + std::to_string(max_bytes) +
                           ") exceeded without seeing terminator");
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            throw TcpError("read_until: timeout waiting for \"" + terminator + "\"");
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        std::size_t got = read_some(
            MutByteSpan{scratch,
                        std::min<std::size_t>(sizeof(scratch), max_bytes - acc.size())},
            remaining);
        if (got == 0) {
            throw TcpError("read_until: peer closed before terminator");
        }
        acc.append(reinterpret_cast<const char*>(scratch), got);
    }
}

}  // namespace asterx
