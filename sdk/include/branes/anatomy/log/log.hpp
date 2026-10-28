#pragma once
/// @file log.hpp
/// @brief The replay log: LogWriter and LogReader (SYS-08, SW-03).
///
/// File format (all integers little-endian):
///
///   magic        8 bytes  "BRLOG\0\0\0"
///   version      u32      kLogVersion
///   header_size  u32      size of the encoded LogHeader that follows
///   header       LogHeader fields, strings as u32 length + bytes
///   records      repeated until end of file:
///                  channel u16, payload_size u32, t i64 [ns], payload
///
/// The header embeds the world (racing-drone: the course) and vehicle files, so a
/// log replays on its own (racing-drone docs/adr/0001-baseline-composition-framework.md,
/// harness inputs).
/// Phase 0 writes synchronously, off the hot path; the per-channel ring
/// buffers and recorder thread of ADR 0006 replace the write path in Phase 1+.

#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::log {

inline constexpr std::uint32_t kLogVersion = 2;  ///< 2: the reference channel

/// Record channels. Inputs (< 16) are what the harness feeds the stack;
/// outputs (≥ 16) are what the stack produced. Sim channels are inputs the
/// SITL harness records for scoring and for SITL-only slots; the stack does not
/// consume them yet, and replay copies them through.
///
/// The enumerators are the framework's channels, the same for every mission.
/// A mission adds its own channels at ids no enumerator uses (racing-drone: 5,
/// oracle_gates; branes/anatomy/log/channels.hpp), and names them to the
/// LogReader. The ids never change meaning, so existing logs read unchanged.
enum class Channel : std::uint16_t {
    imu = 1,
    camera = 2,
    tick = 3,
    truth = 4,       ///< TruthState (SITL)
    sim_config = 6,  ///< the run's drawn randomization, as YAML text (SITL)
    state = 16,
    command = 17,
    reference = 18,  ///< the planner's Reference, the controller's input
};

[[nodiscard]] constexpr bool is_output(Channel c) noexcept {
    return static_cast<std::uint16_t>(c) >= 16;
}

/// True if `c` is one of the framework's channels (an enumerator of Channel).
/// The switch lists every enumerator and has no default, so a new enumerator
/// left out of it is a -Wswitch error (warnings are errors), and this check
/// cannot drift from the enum.
[[nodiscard]] constexpr bool is_framework(std::uint16_t c) noexcept {
    switch (static_cast<Channel>(c)) {
    case Channel::imu:
    case Channel::camera:
    case Channel::tick:
    case Channel::truth:
    case Channel::sim_config:
    case Channel::state:
    case Channel::command:
    case Channel::reference:
        return true;
    }
    return false;
}

/// True if `c` is a framework channel or one of `mission`'s.
[[nodiscard]] constexpr bool is_known(std::uint16_t c, std::span<const Channel> mission) noexcept {
    if (is_framework(c)) {
        return true;
    }
    for (const Channel m : mission) {
        if (static_cast<std::uint16_t>(m) == c) {
            return true;
        }
    }
    return false;
}

/// Run provenance (ADR 0001): what produced the log, and from which inputs.
struct LogHeader {
    std::string baseline_name;
    std::string composition_hash;
    std::string world_hash;  ///< of the world file (racing-drone: the course)
    std::string vehicle_hash;
    std::string scenario_hash{"none"};  ///< "none" when no scenario file was used (flight)
    bool reference{true};               ///< false for what-if replays with overridden inputs
    std::string world_yaml;             ///< the world file (racing-drone: the course), verbatim
    std::string vehicle_yaml;           ///< the vehicle file, verbatim

    friend bool operator==(const LogHeader&, const LogHeader&) = default;
};

struct LogRecord {
    Channel channel{Channel::tick};
    Timestamp t;
    std::vector<std::byte> payload;

    friend bool operator==(const LogRecord&, const LogRecord&) = default;
};

struct LogError {
    std::string path;
    std::string message;
};

/// Appends records to a new log file.
///
/// Model
/// -----
///   out_         : std::ofstream [owned] — the file
///   path_        : the file's path (for errors)
///   open_        : bool — records may be appended
///   last_t_      : per-channel time of the last record
///   records_     : number of records written
///
/// Invariants
/// ----------
///   I1: the header was written exactly once, by open()       — constrains out_
///   I2: records are appended only while open_                — constrains open_
///   I3: within a channel, record times never decrease        — constrains last_t_
class LogWriter {
public:
    /// Create `path` (truncating it) and write the header.
    /// Post (on success): is_open(), record_count() == 0, I1.
    [[nodiscard]] static result<LogWriter, LogError> open(const std::filesystem::path& path, const LogHeader& header);

    /// Post: `other` is closed (it no longer owns the file).
    LogWriter(LogWriter&& other) noexcept
        : out_(std::move(other.out_)), path_(std::move(other.path_)), open_(std::exchange(other.open_, false)),
          last_t_(std::move(other.last_t_)), records_(other.records_) {}
    LogWriter& operator=(LogWriter&& other) noexcept {
        if (this != &other) {
            if (open_) {
                (void)close();
            }
            out_ = std::move(other.out_);
            path_ = std::move(other.path_);
            open_ = std::exchange(other.open_, false);
            last_t_ = std::move(other.last_t_);
            records_ = other.records_;
        }
        return *this;
    }
    LogWriter(const LogWriter&) = delete;
    LogWriter& operator=(const LogWriter&) = delete;
    ~LogWriter();

    /// Append one record. Pre: is_open(); t ≥ the channel's previous record time (I3).
    /// I/O errors are latched and reported by close().
    void write(Channel channel, Timestamp t, std::span<const std::byte> payload);

    /// Flush and close. Post: !is_open(). Returns the first I/O error, if any.
    result<ok_t, LogError> close();

    [[nodiscard]] bool is_open() const noexcept {
        return open_;
    }
    [[nodiscard]] std::uint64_t record_count() const noexcept {
        return records_;
    }

private:
    LogWriter(std::ofstream out, std::filesystem::path path) noexcept
        : out_(std::move(out)), path_(std::move(path)), open_(true) {}

    std::ofstream out_;
    std::filesystem::path path_;
    bool open_{false};
    std::vector<std::pair<Channel, std::int64_t>> last_t_;
    std::uint64_t records_{0};
};

/// Reads a log sequentially.
///
/// Model: in_ [owned] — the file; header_ — the decoded header; mission_ — the
/// mission's channels.
/// Invariants:
///   I1: between calls, the read position is on a record boundary (or at the end)
///   I2: mission_ holds no framework channel and no id twice
class LogReader {
public:
    /// Open `path` and validate the magic, version, and header. `mission` names
    /// the mission's channels (racing-drone: log::kRacingChannels); a record on any
    /// other channel that is not a framework channel is an error.
    /// Pre-checked (an error, not UB): `mission` satisfies I2.
    [[nodiscard]] static result<LogReader, LogError> open(const std::filesystem::path& path,
                                                          std::span<const Channel> mission);

    [[nodiscard]] const LogHeader& header() const noexcept {
        return header_;
    }

    /// The next record; nullopt at a clean end of file; an error for a truncated
    /// record or an unknown channel. Preserves I1 and I2.
    [[nodiscard]] result<std::optional<LogRecord>, LogError> next();

private:
    LogReader(std::ifstream in, std::filesystem::path path, LogHeader header, std::vector<Channel> mission) noexcept
        : in_(std::move(in)), path_(std::move(path)), header_(std::move(header)), mission_(std::move(mission)) {}

    std::ifstream in_;
    std::filesystem::path path_;
    LogHeader header_;
    std::vector<Channel> mission_;
};

}  // namespace branes::anatomy::log
