#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <array>
#include <cassert>

namespace branes::anatomy::log {

namespace {

constexpr std::array<char, 8> kMagic = {'B', 'R', 'L', 'O', 'G', '\0', '\0', '\0'};
constexpr std::size_t kRecordHeaderSize = 2 + 4 + 8;
// A frame at 4K RGB is ~25 MB; anything far larger is a corrupt size field.
constexpr std::uint32_t kMaxPayload = 256u * 1024u * 1024u;

std::vector<std::byte> encode_header(const LogHeader& h) {
    ByteWriter w;
    w.str(h.baseline_name);
    w.str(h.composition_hash);
    w.str(h.world_hash);
    w.str(h.vehicle_hash);
    w.str(h.scenario_hash);
    w.boolean(h.reference);
    w.str(h.world_yaml);
    w.str(h.vehicle_yaml);
    return w.take();
}

std::optional<LogHeader> decode_header(std::span<const std::byte> b) {
    ByteReader r(b);
    LogHeader h;
    h.baseline_name = r.str();
    h.composition_hash = r.str();
    h.world_hash = r.str();
    h.vehicle_hash = r.str();
    h.scenario_hash = r.str();
    h.reference = r.boolean();
    h.world_yaml = r.str();
    h.vehicle_yaml = r.str();
    if (!r.done()) {
        return std::nullopt;
    }
    return h;
}

void write_bytes(std::ofstream& out, std::span<const std::byte> b) {
    out.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

bool read_bytes(std::ifstream& in, std::span<std::byte> b) {
    in.read(reinterpret_cast<char*>(b.data()), static_cast<std::streamsize>(b.size()));
    return static_cast<std::size_t>(in.gcount()) == b.size();
}

}  // namespace

// ── LogWriter ───────────────────────────────────────────────────────

result<LogWriter, LogError> LogWriter::open(const std::filesystem::path& path, const LogHeader& header) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return LogError{path.string(), "cannot create the log file"};
    }
    const std::vector<std::byte> h = encode_header(header);
    ByteWriter prefix;
    for (const char c : kMagic) {
        prefix.u8(static_cast<std::uint8_t>(c));
    }
    prefix.u32(kLogVersion);
    prefix.u32(static_cast<std::uint32_t>(h.size()));
    write_bytes(out, prefix.data());
    write_bytes(out, h);  // I1: the header, once
    if (!out) {
        return LogError{path.string(), "cannot write the log header"};
    }
    return LogWriter(std::move(out), path);
}

LogWriter::~LogWriter() {
    if (open_) {
        (void)close();
    }
}

void LogWriter::write(Channel channel, Timestamp t, std::span<const std::byte> payload) {
    assert(open_);  // Pre / I2
    auto it = last_t_.begin();
    while (it != last_t_.end() && it->first != channel) {
        ++it;
    }
    if (it == last_t_.end()) {
        last_t_.emplace_back(channel, t.ns);
    } else {
        assert(it->second <= t.ns);  // Pre / I3
        it->second = t.ns;
    }

    ByteWriter rec;
    rec.u16(static_cast<std::uint16_t>(channel));
    rec.u32(static_cast<std::uint32_t>(payload.size()));
    rec.i64(t.ns);
    write_bytes(out_, rec.data());
    write_bytes(out_, payload);
    ++records_;
}

result<ok_t, LogError> LogWriter::close() {
    if (!open_) {
        return ok;
    }
    open_ = false;
    out_.flush();
    const bool good = static_cast<bool>(out_);
    out_.close();
    if (!good || out_.fail()) {
        return LogError{path_.string(), "write error"};
    }
    return ok;
}

// ── LogReader ───────────────────────────────────────────────────────

result<LogReader, LogError> LogReader::open(const std::filesystem::path& path, std::span<const Channel> mission) {
    for (std::size_t i = 0; i < mission.size(); ++i) {
        const auto id = static_cast<std::uint16_t>(mission[i]);
        if (is_framework(id)) {
            return LogError{path.string(), "mission channel " + std::to_string(id) + " is a framework channel"};  // I2
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (mission[j] == mission[i]) {
                return LogError{path.string(), "mission channel " + std::to_string(id) + " named twice"};  // I2
            }
        }
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return LogError{path.string(), "cannot open the log file"};
    }
    std::array<std::byte, 16> prefix{};
    if (!read_bytes(in, prefix)) {
        return LogError{path.string(), "file too short for a log"};
    }
    for (std::size_t i = 0; i < kMagic.size(); ++i) {
        if (prefix[i] != static_cast<std::byte>(kMagic[i])) {
            return LogError{path.string(), "not a log (bad magic)"};
        }
    }
    ByteReader r(std::span<const std::byte>(prefix).subspan(8));
    const std::uint32_t version = r.u32();
    const std::uint32_t header_size = r.u32();
    if (version != kLogVersion) {
        return LogError{path.string(), "unsupported log version " + std::to_string(version)};
    }
    if (header_size > kMaxPayload) {
        return LogError{path.string(), "corrupt header size"};
    }
    std::vector<std::byte> h(header_size);
    if (!read_bytes(in, h)) {
        return LogError{path.string(), "truncated header"};
    }
    auto header = decode_header(h);
    if (!header) {
        return LogError{path.string(), "corrupt header"};
    }
    return LogReader(std::move(in), path, std::move(*header), std::vector<Channel>(mission.begin(), mission.end()));
}

result<std::optional<LogRecord>, LogError> LogReader::next() {
    std::array<std::byte, kRecordHeaderSize> rh{};
    in_.read(reinterpret_cast<char*>(rh.data()), static_cast<std::streamsize>(rh.size()));
    const auto got = static_cast<std::size_t>(in_.gcount());
    if (got == 0 && in_.eof()) {
        return std::optional<LogRecord>{};  // clean end of file (I1)
    }
    if (got != rh.size()) {
        return LogError{path_.string(), "truncated record header"};
    }
    ByteReader r(rh);
    const std::uint16_t channel = r.u16();
    const std::uint32_t size = r.u32();
    const std::int64_t t = r.i64();
    if (!is_known(channel, mission_)) {
        return LogError{path_.string(), "unknown channel " + std::to_string(channel)};
    }
    if (size > kMaxPayload) {
        return LogError{path_.string(), "corrupt record size"};
    }
    LogRecord rec{static_cast<Channel>(channel), Timestamp{t}, std::vector<std::byte>(size)};
    if (!read_bytes(in_, rec.payload)) {
        return LogError{path_.string(), "truncated record payload"};
    }
    return std::optional<LogRecord>{std::move(rec)};
}

}  // namespace branes::anatomy::log
