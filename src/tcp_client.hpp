// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "asterx/byte_view.hpp"

namespace asterx {

class TcpError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Blocking BSD socket. One connection per instance. Not thread-safe; the
// recorder runs everything on a single thread.
class TcpClient {
public:
    TcpClient() noexcept = default;
    ~TcpClient();

    TcpClient(const TcpClient&)            = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    // Open a TCP connection. host may be an IPv4 literal or a DNS name.
    // Throws TcpError on failure.
    void connect(const std::string& host,
                 std::uint16_t      port,
                 std::chrono::milliseconds connect_timeout);

    void close() noexcept;
    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

    // Write exactly buf.size() bytes; throws on short write or error.
    void write_all(ConstByteSpan buf);

    // Convenience: write a string verbatim.
    void write_all(const std::string& s);

    // Read up to buf.size() bytes; returns number read (>0). Returns 0 only
    // on clean peer close. Throws TcpError on socket error or timeout.
    // The supplied io_timeout applies to a single read() call.
    [[nodiscard]] std::size_t read_some(MutByteSpan buf,
                                        std::chrono::milliseconds io_timeout);

    // Read exactly buf.size() bytes; throws on short read.
    void read_exact(MutByteSpan buf, std::chrono::milliseconds io_timeout);

    // Read until either:
    //   * `terminator` substring observed in the accumulated buffer, OR
    //   * total_timeout exceeded (throws), OR
    //   * `max_bytes` reached (throws).
    // Returns the full accumulated string including the terminator.
    [[nodiscard]] std::string read_until(const std::string& terminator,
                                         std::chrono::milliseconds total_timeout,
                                         std::size_t               max_bytes = 65536);

private:
    int fd_{-1};
};

}  // namespace asterx
