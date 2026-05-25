// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>

namespace asterx {

// C++17 stand-in for std::span<uint8_t> / std::span<const uint8_t>.
// Non-owning pointer + size. Trivially copyable.

class MutByteSpan {
public:
    constexpr MutByteSpan() noexcept = default;
    constexpr MutByteSpan(std::uint8_t* data, std::size_t size) noexcept
        : data_(data), size_(size) {}

    [[nodiscard]] constexpr std::uint8_t* data() const noexcept { return data_; }
    [[nodiscard]] constexpr std::size_t   size() const noexcept { return size_; }
    [[nodiscard]] constexpr bool          empty() const noexcept { return size_ == 0; }
    [[nodiscard]] constexpr std::uint8_t* begin() const noexcept { return data_; }
    [[nodiscard]] constexpr std::uint8_t* end()   const noexcept { return data_ + size_; }
    constexpr std::uint8_t& operator[](std::size_t i) const noexcept { return data_[i]; }

private:
    std::uint8_t* data_{nullptr};
    std::size_t   size_{0};
};

class ConstByteSpan {
public:
    constexpr ConstByteSpan() noexcept = default;
    constexpr ConstByteSpan(const std::uint8_t* data, std::size_t size) noexcept
        : data_(data), size_(size) {}
    constexpr ConstByteSpan(MutByteSpan s) noexcept
        : data_(s.data()), size_(s.size()) {}

    [[nodiscard]] constexpr const std::uint8_t* data() const noexcept { return data_; }
    [[nodiscard]] constexpr std::size_t        size() const noexcept { return size_; }
    [[nodiscard]] constexpr bool               empty() const noexcept { return size_ == 0; }
    [[nodiscard]] constexpr const std::uint8_t* begin() const noexcept { return data_; }
    [[nodiscard]] constexpr const std::uint8_t* end()   const noexcept { return data_ + size_; }
    constexpr const std::uint8_t& operator[](std::size_t i) const noexcept { return data_[i]; }

private:
    const std::uint8_t* data_{nullptr};
    std::size_t         size_{0};
};

}  // namespace asterx
