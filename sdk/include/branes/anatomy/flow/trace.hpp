#pragma once
/// @file trace.hpp
/// @brief The edge trace: every value a stack's firings write, as a file
/// beside the log (docs/plan/flow-testing, T4).
///
/// A stack's observer (flow/builder.hpp) is handed each firing once it has
/// written its out ports. trace_writer writes one record per out port, so a
/// trace holds every edge of a run, not only the outputs the log holds, and
/// two traces show the first edge on which two runs differ (compare_traces,
/// and the tool trace_compare). The log, its format and its gates don't
/// change: a trace is written only when asked for.
///
/// File format (all integers little-endian, strings as u32 length + bytes):
///
///   magic        8 bytes  "BRTRACE\0"
///   version      u32      kTraceVersion
///   header_size  u32      size of the header that follows
///   header       the mission, and the graph's hash
///   records      repeated until end of file: record_size u32, then
///                  region str (its trigger), firing u32 (its index there),
///                  op str, handler str, composite str (`c.h`; empty in research),
///                  port str, t i64 [ns], coded u8, and if coded, the payload str
///
/// A record's payload is its message as the mission's codec encodes it
/// (trace_codecs). A message of a type with no codec is traced without it:
/// its edge and time are kept, so a comparison still names it.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/log/compare.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace branes::anatomy::flow {

inline constexpr std::uint32_t kTraceVersion = 1;

/// The encoders a trace writes messages with, by message type: the mission's
/// log codecs (racing-drone: codec::encode), added for the types it traces.
class trace_codecs {
public:
    using encode_fn = std::function<std::vector<std::byte>(const void*)>;

    /// Encode messages of type M with `encode(const M&) -> std::vector<std::byte>`.
    template <class M, class F>
    void add(F encode) {
        fns_[std::type_index(typeid(M))] = [f = std::move(encode)](const void* m) {
            return f(*static_cast<const M*>(m));
        };
    }

    /// The encoder of `type`, or null if it has none.
    [[nodiscard]] const encode_fn* find(std::type_index type) const {
        const auto it = fns_.find(type);
        return it == fns_.end() ? nullptr : &it->second;
    }

private:
    std::unordered_map<std::type_index, encode_fn> fns_;
};

struct trace_header {
    std::string mission;
    std::string graph_hash;

    friend bool operator==(const trace_header&, const trace_header&) = default;
};

/// One edge's value, as a firing wrote it.
struct trace_record {
    std::string region;  ///< the region's trigger
    std::uint32_t firing = 0;
    std::string op;
    std::string handler;
    std::string composite;  ///< `c.h`, the composite handler it implements; empty in research
    std::string port;
    Timestamp t;
    std::optional<std::vector<std::byte>> payload;  ///< nullopt: its type has no codec

    /// The edge, as `region[firing].port`.
    [[nodiscard]] std::string edge() const {
        return region + "[" + std::to_string(firing) + "]." + port;
    }

    friend bool operator==(const trace_record&, const trace_record&) = default;
};

/// Writes a stack's firings to a trace file. Hand it to the stack as its
/// observer: stack.observe([&w](const fired& f) { w.write(f); }).
///
/// Invariants: the header was written once, by open(); records are written
/// only while open.
class trace_writer {
public:
    /// Create `path` (truncating it), and write the header of graph `g`, the
    /// graph of the stack it will observe.
    [[nodiscard]] static result<trace_writer, log::LogError>
    open(const std::filesystem::path& path, const Graph& g, trace_codecs codecs);

    trace_writer(trace_writer&& other) noexcept;
    trace_writer& operator=(trace_writer&& other) noexcept;
    trace_writer(const trace_writer&) = delete;
    trace_writer& operator=(const trace_writer&) = delete;
    ~trace_writer();

    /// One record per out port of `f`. Pre: is_open(); `f` is a firing of the graph open() was given.
    /// I/O errors are latched and reported by close().
    void write(const fired& f);

    /// Flush and close. Returns the first I/O error, if any.
    result<ok_t, log::LogError> close();

    [[nodiscard]] bool is_open() const noexcept {
        return open_;
    }
    [[nodiscard]] std::uint64_t record_count() const noexcept {
        return records_;
    }

private:
    trace_writer(std::ofstream out, std::filesystem::path path, Graph g, trace_codecs codecs);

    std::ofstream out_;
    std::filesystem::path path_;
    Graph graph_;
    trace_codecs codecs_;
    bool open_ = false;
    std::uint64_t records_ = 0;
};

/// Reads a trace sequentially.
class trace_reader {
public:
    /// Open `path`, and read its magic, version and header.
    [[nodiscard]] static result<trace_reader, log::LogError> open(const std::filesystem::path& path);

    [[nodiscard]] const trace_header& header() const noexcept {
        return header_;
    }

    /// The next record; nullopt at a clean end of file; an error for a truncated or corrupt record.
    [[nodiscard]] result<std::optional<trace_record>, log::LogError> next();

private:
    trace_reader(std::ifstream in, std::filesystem::path path, trace_header header) noexcept
        : in_(std::move(in)), path_(std::move(path)), header_(std::move(header)) {}

    std::ifstream in_;
    std::filesystem::path path_;
    trace_header header_;
};

/// Compare the traces at `a` and `b`, record by record: the first record whose
/// edge, firing, time or payload differs, or a trace with more records, is the
/// difference, named by its edge. The headers aren't compared: two baselines'
/// traces, of different graphs, are compared to find where they first part.
/// An error is a trace that can't be read.
[[nodiscard]] result<log::Comparison, log::LogError> compare_traces(const std::filesystem::path& a,
                                                                    const std::filesystem::path& b);

}  // namespace branes::anatomy::flow
