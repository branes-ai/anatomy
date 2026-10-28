#pragma once
/// @file bytes.hpp
/// @brief ByteWriter / ByteReader: explicit little-endian encoding for the log.
///
/// Structs are never written with memcpy: padding bytes are indeterminate, and
/// identical inputs must produce identical bytes (SW-03). Every field is
/// written explicitly, floating point by its IEEE-754 bit pattern.

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::anatomy {

/// Appends little-endian fields to a byte buffer.
class ByteWriter {
public:
    void u8(std::uint8_t v) {
        buf_.push_back(std::byte{v});
    }
    void u16(std::uint16_t v) {
        put(v, 2);
    }
    void u32(std::uint32_t v) {
        put(v, 4);
    }
    void u64(std::uint64_t v) {
        put(v, 8);
    }
    void i32(std::int32_t v) {
        put(static_cast<std::uint32_t>(v), 4);
    }
    void i64(std::int64_t v) {
        put(static_cast<std::uint64_t>(v), 8);
    }
    void f64(double v) {
        put(std::bit_cast<std::uint64_t>(v), 8);
    }
    void boolean(bool v) {
        u8(v ? 1 : 0);
    }
    void bytes(std::span<const std::byte> b) {
        buf_.insert(buf_.end(), b.begin(), b.end());
    }
    /// A u32 length followed by the characters.
    void str(std::string_view s) {
        u32(static_cast<std::uint32_t>(s.size()));
        for (const char c : s) {
            buf_.push_back(static_cast<std::byte>(c));
        }
    }

    [[nodiscard]] const std::vector<std::byte>& data() const noexcept {
        return buf_;
    }
    [[nodiscard]] std::vector<std::byte> take() noexcept {
        return std::move(buf_);
    }

private:
    void put(std::uint64_t v, int n) {
        for (int i = 0; i < n; ++i) {
            buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFu));
        }
    }

    std::vector<std::byte> buf_;
};

/// Reads little-endian fields from a byte span. A read past the end sets
/// failed() and returns zero; callers check failed() once, at the end.
///
/// Invariants: I1 pos_ ≤ data_.size().
class ByteReader {
public:
    explicit ByteReader(std::span<const std::byte> data) noexcept : data_(data) {}

    std::uint8_t u8() noexcept {
        return static_cast<std::uint8_t>(get(1));
    }
    std::uint16_t u16() noexcept {
        return static_cast<std::uint16_t>(get(2));
    }
    std::uint32_t u32() noexcept {
        return static_cast<std::uint32_t>(get(4));
    }
    std::uint64_t u64() noexcept {
        return get(8);
    }
    std::int32_t i32() noexcept {
        return static_cast<std::int32_t>(static_cast<std::uint32_t>(get(4)));
    }
    std::int64_t i64() noexcept {
        return static_cast<std::int64_t>(get(8));
    }
    double f64() noexcept {
        return std::bit_cast<double>(get(8));
    }
    bool boolean() noexcept {
        return u8() != 0;
    }
    /// The next n bytes, or an empty span (and failed()) if fewer remain.
    std::span<const std::byte> bytes(std::size_t n) noexcept {
        if (n > remaining()) {
            failed_ = true;
            pos_ = data_.size();
            return {};
        }
        const auto out = data_.subspan(pos_, n);
        pos_ += n;
        return out;
    }
    std::string str() {
        const std::uint32_t n = u32();
        const auto b = bytes(n);
        std::string s(b.size(), '\0');
        for (std::size_t i = 0; i < b.size(); ++i) {
            s[i] = static_cast<char>(b[i]);
        }
        return s;
    }

    [[nodiscard]] std::size_t remaining() const noexcept {
        return data_.size() - pos_;
    }
    [[nodiscard]] bool failed() const noexcept {
        return failed_;
    }
    /// True if every byte was consumed and no read failed.
    [[nodiscard]] bool done() const noexcept {
        return !failed_ && remaining() == 0;
    }

private:
    std::uint64_t get(int n) noexcept {
        const auto b = bytes(static_cast<std::size_t>(n));
        std::uint64_t v = 0;
        for (std::size_t i = 0; i < b.size(); ++i) {
            v |= static_cast<std::uint64_t>(std::to_integer<std::uint8_t>(b[i])) << (8 * i);
        }
        return v;
    }

    std::span<const std::byte> data_;
    std::size_t pos_{0};
    bool failed_{false};
};

}  // namespace branes::anatomy
