#include <branes/anatomy/log/compare.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <utility>
#include <vector>

namespace branes::anatomy::log {

namespace {

constexpr std::size_t kPrefixSize = 8 + 4 + 4;  // magic, version, header size
constexpr std::size_t kRecordHeaderSize = 2 + 4 + 8;

/// One record's framing and payload, as stored.
struct RawRecord {
    std::uint16_t channel{0};
    std::int64_t t{0};
    std::vector<std::byte> payload;
};

/// A log's records, read past its header, on any channel.
class RawRecords {
public:
    /// Pre: `path` opened as a log (LogReader::open), so its prefix is whole.
    [[nodiscard]] static result<RawRecords, LogError> open(const std::filesystem::path& path) {
        std::ifstream in(path, std::ios::binary);
        std::array<std::byte, kPrefixSize> prefix{};
        in.read(reinterpret_cast<char*>(prefix.data()), static_cast<std::streamsize>(prefix.size()));
        if (static_cast<std::size_t>(in.gcount()) != prefix.size()) {
            return LogError{path.string(), "file too short for a log"};
        }
        ByteReader r(std::span<const std::byte>(prefix).subspan(12));
        in.seekg(static_cast<std::streamoff>(kPrefixSize + r.u32()));
        if (!in) {
            return LogError{path.string(), "truncated header"};
        }
        return RawRecords(std::move(in), path);
    }

    /// The next record; nullopt at a clean end of file.
    [[nodiscard]] result<std::optional<RawRecord>, LogError> next() {
        std::array<std::byte, kRecordHeaderSize> rh{};
        in_.read(reinterpret_cast<char*>(rh.data()), static_cast<std::streamsize>(rh.size()));
        const auto got = static_cast<std::size_t>(in_.gcount());
        if (got == 0 && in_.eof()) {
            return std::optional<RawRecord>{};
        }
        if (got != rh.size()) {
            return LogError{path_.string(), "truncated record header"};
        }
        ByteReader r(rh);
        RawRecord rec;
        rec.channel = r.u16();
        const std::uint32_t size = r.u32();
        rec.t = r.i64();
        rec.payload.resize(size);
        in_.read(reinterpret_cast<char*>(rec.payload.data()), static_cast<std::streamsize>(size));
        if (static_cast<std::size_t>(in_.gcount()) != size) {
            return LogError{path_.string(), "truncated record payload"};
        }
        return std::optional<RawRecord>{std::move(rec)};
    }

private:
    RawRecords(std::ifstream in, std::filesystem::path path) noexcept : in_(std::move(in)), path_(std::move(path)) {}

    std::ifstream in_;
    std::filesystem::path path_;
};

/// The header fields as text, in kHeaderFields' order.
std::array<std::string, kHeaderFields.size()> fields_of(const LogHeader& h) {
    return {h.baseline_name,
            h.composition_hash,
            h.world_hash,
            h.vehicle_hash,
            h.scenario_hash,
            h.reference ? "true" : "false",
            h.world_yaml,
            h.vehicle_yaml};
}

std::string describe(const RawRecord& r) {
    return "channel " + std::to_string(r.channel) + " at t=" + std::to_string(r.t) + " ns";
}

}  // namespace

result<Comparison, LogError>
compare(const std::filesystem::path& a, const std::filesystem::path& b, std::span<const std::string_view> except) {
    for (const std::string_view e : except) {
        if (std::find(kHeaderFields.begin(), kHeaderFields.end(), e) == kHeaderFields.end()) {
            return LogError{a.string(), "'" + std::string(e) + "' is not a header field"};
        }
    }
    auto reader_a = LogReader::open(a, {});
    if (!reader_a) {
        return reader_a.error();
    }
    auto reader_b = LogReader::open(b, {});
    if (!reader_b) {
        return reader_b.error();
    }

    Comparison out;
    const auto fa = fields_of(reader_a->header());
    const auto fb = fields_of(reader_b->header());
    for (std::size_t i = 0; i < kHeaderFields.size(); ++i) {
        if (std::find(except.begin(), except.end(), kHeaderFields[i]) != except.end() || fa[i] == fb[i]) {
            continue;
        }
        const bool text = kHeaderFields[i] == "world_yaml" || kHeaderFields[i] == "vehicle_yaml";
        out.difference = "the header's " + std::string(kHeaderFields[i]) + " differs" +
                         (text ? std::string() : ": '" + fa[i] + "' and '" + fb[i] + "'");
        return out;
    }

    auto records_a = RawRecords::open(a);
    if (!records_a) {
        return records_a.error();
    }
    auto records_b = RawRecords::open(b);
    if (!records_b) {
        return records_b.error();
    }
    for (;;) {
        auto ra = records_a->next();
        if (!ra) {
            return ra.error();
        }
        auto rb = records_b->next();
        if (!rb) {
            return rb.error();
        }
        if (!*ra || !*rb) {
            if (*ra || *rb) {
                const std::string longer = *ra ? a.string() : b.string();
                out.difference = longer + " has more records: record " + std::to_string(out.records) + " is " +
                                 describe(*ra ? **ra : **rb);
            }
            return out;
        }
        const RawRecord& x = **ra;
        const RawRecord& y = **rb;
        if (x.channel != y.channel || x.t != y.t) {
            out.difference = "record " + std::to_string(out.records) + " is " + describe(x) + " in one and " +
                             describe(y) + " in the other";
            return out;
        }
        if (x.payload != y.payload) {
            out.difference = "record " + std::to_string(out.records) + " (" + describe(x) + ") has another payload";
            return out;
        }
        ++out.records;
    }
}

}  // namespace branes::anatomy::log
