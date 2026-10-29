#pragma once
/// @file dump.hpp
/// @brief dump_records(): a log's records as CSV and YAML files, for plotting (log_dump).
///
/// Each channel goes
/// to its own file; the framework's channels have fixed formats (below), and a
/// mission adds a ChannelFormat for each of its own (racing-drone: oracle_gates,
/// apps/log_dump/main.cpp).
///
/// Every CSV row starts with t_ns (the record time, exact) and t_s (the same
/// in seconds). Doubles are printed in shortest round-trip form, so a value
/// read back is the logged value. Attitudes are the logged quaternion (body →
/// world) plus roll, pitch, yaw [rad] in the Z-Y-X convention; state.csv
/// carries the covariance diagonal only, and camera.csv the frame header
/// without pixels. reference.csv has one row per reference sample, and
/// sim_config.yaml one YAML document per record.

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace branes::anatomy::log {

namespace csv {

inline void put(std::string& row, double v) {
    char buf[32];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof buf, v);
    row.push_back(',');
    row.append(buf, ec == std::errc{} ? end : buf);
}

inline void put(std::string& row, std::int64_t v) {
    row.push_back(',');
    row += std::to_string(v);
}

inline void put(std::string& row, std::uint64_t v) {
    row.push_back(',');
    row += std::to_string(v);
}

inline void put(std::string& row, const Vec3<double>& v) {
    for (const double c : v) {
        put(row, c);
    }
}

/// The quaternion, then roll, pitch, yaw (Z-Y-X) of the body → world rotation.
inline void put(std::string& row, const Quat<double>& q) {
    put(row, q.w);
    put(row, q.x);
    put(row, q.y);
    put(row, q.z);
    put(row, std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)));
    put(row, std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)));
    put(row, std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
}

/// t_ns, t_s: the record time.
inline std::string start(Timestamp t) {
    std::string row = std::to_string(t.ns);
    put(row, static_cast<double>(t.ns) / 1e9);
    return row;
}

inline std::string vec3(std::string_view name) {
    std::string s;
    for (const char axis : {'x', 'y', 'z'}) {
        s += ',';
        s += name;
        s += '_';
        s += axis;
    }
    return s;
}

inline std::string quat(std::string_view name) {
    std::string s;
    for (const char* part : {"_w", "_x", "_y", "_z"}) {
        s += ',';
        s += name;
        s += part;
    }
    return s + ",roll,pitch,yaw";
}

}  // namespace csv

/// How a mission's own channel is dumped: its file, its CSV header row, and
/// the rows one record contributes (nullopt when the payload does not decode;
/// `count` is set to the number of rows).
struct ChannelFormat {
    Channel channel;
    std::string file_name;
    std::string columns;
    std::optional<std::string> (*rows)(const LogRecord& r, std::uint64_t& count);
};

namespace framework {

using namespace csv;

/// The file a framework channel is dumped to.
[[nodiscard]] inline const char* file_name(Channel c) {
    switch (c) {
    case Channel::imu:
        return "imu.csv";
    case Channel::camera:
        return "camera.csv";
    case Channel::tick:
        return "tick.csv";
    case Channel::truth:
        return "truth.csv";
    case Channel::sim_config:
        return "sim_config.yaml";
    case Channel::state:
        return "state.csv";
    case Channel::reference:
        return "reference.csv";
    case Channel::command:
        return "command.csv";
    }
    return "unknown.csv";
}

/// The CSV header row of a framework channel (empty for sim_config, a YAML file).
[[nodiscard]] inline std::string columns(Channel c) {
    const std::string t = "t_ns,t_s";
    switch (c) {
    case Channel::imu:
        return t + ",stamp_ns" + vec3("gyro") + vec3("accel") + ",saturated,temperature";
    case Channel::camera:
        return t + ",t_mid_ns,exposure_ns,row_readout_ns,width,height,stride,format,corrupted,bytes";
    case Channel::tick:
        return t;
    case Channel::truth:
        return t + ",stamp_ns" + vec3("p") + vec3("v") + quat("q") + vec3("w") +
               ",rotor_0,rotor_1,rotor_2,rotor_3,on_ground";
    case Channel::state: {
        std::string s =
            t + ",t_valid_ns" + vec3("p") + vec3("v") + quat("q") + vec3("w") + vec3("accel_bias") + vec3("gyro_bias");
        for (std::size_t i = 0; i < kErrorStateDim; ++i) {
            s += ",var_" + std::to_string(i);
        }
        return s;
    }
    case Channel::reference:
        return t + ",count,index,sample_t_ns" + vec3("p") + vec3("v") + vec3("a") + ",yaw,progress";
    case Channel::command:
        return t + ",stamp_ns,kind,thrust" + vec3("w") + ",motor_0,motor_1,motor_2,motor_3";
    case Channel::sim_config:
        break;
    }
    return {};
}

/// The rows one record contributes to its channel's file, or nullopt when the
/// payload does not decode.
[[nodiscard]] inline std::optional<std::string> rows_of(const LogRecord& r, std::uint64_t& count) {
    std::string row = start(r.t);
    count = 1;
    switch (r.channel) {
    case Channel::imu: {
        const auto x = codec::decode_imu(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        put(row, x->gyro);
        put(row, x->accel);
        put(row, std::uint64_t{x->saturated});
        put(row, x->temperature);
        break;
    }
    case Channel::camera: {
        const auto x = codec::decode_frame(r.payload);
        if (!x) {
            return std::nullopt;
        }
        const FrameHeader& h = x->header;
        put(row, h.t_mid.ns);
        put(row, h.exposure_ns);
        put(row, h.row_readout_ns);
        put(row, std::uint64_t{h.width});
        put(row, std::uint64_t{h.height});
        put(row, std::uint64_t{h.stride});
        row += h.format == PixelFormat::rgb8 ? ",rgb8" : ",mono8";
        put(row, std::uint64_t{h.corrupted});
        put(row, std::uint64_t{x->bytes.size()});
        break;
    }
    case Channel::tick:
        break;
    case Channel::truth: {
        const auto x = codec::decode_truth(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        put(row, x->position);
        put(row, x->velocity);
        put(row, x->attitude);
        put(row, x->body_rates);
        for (const double w : x->rotor_speeds) {
            put(row, w);
        }
        put(row, std::uint64_t{x->on_ground});
        break;
    }
    case Channel::state: {
        const auto x = codec::decode_state(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t_valid.ns);
        put(row, x->position);
        put(row, x->velocity);
        put(row, x->attitude);
        put(row, x->body_rates);
        put(row, x->accel_bias);
        put(row, x->gyro_bias);
        for (std::size_t i = 0; i < kErrorStateDim; ++i) {
            put(row, x->covariance[i * kErrorStateDim + i]);
        }
        break;
    }
    case Channel::reference: {
        const auto x = codec::decode_reference(r.payload);
        if (!x) {
            return std::nullopt;
        }
        // One row per sample; an empty reference contributes no row.
        const std::string time = row;
        row.clear();
        for (std::uint32_t i = 0; i < x->count; ++i) {
            const ReferenceSample<double>& s = x->samples[i];
            row += time;
            put(row, std::uint64_t{x->count});
            put(row, std::uint64_t{i});
            put(row, s.t.ns);
            put(row, s.position);
            put(row, s.velocity);
            put(row, s.acceleration);
            put(row, s.yaw);
            put(row, s.progress);
            row += '\n';
        }
        count = x->count;
        return row;
    }
    case Channel::command: {
        const auto x = codec::decode_command(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        row += x->kind == CommandKind::motors ? ",motors" : ",thrust_rates";
        put(row, x->thrust);
        put(row, x->body_rates);
        for (const double m : x->motors) {
            put(row, m);
        }
        break;
    }
    case Channel::sim_config:
        // YAML text: one document per record.
        return std::string("---\n") + std::string(reinterpret_cast<const char*>(r.payload.data()), r.payload.size()) +
               '\n';
    }
    return row + '\n';
}

}  // namespace framework

/// Write `text` to `path`, replacing it. Returns false on an I/O error.
[[nodiscard]] inline bool write_text(const std::filesystem::path& path, std::string_view text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
    return !out.fail();
}

/// One file dump_records() wrote.
struct DumpedFile {
    Channel channel;
    std::string file_name;
    std::filesystem::path path;
    std::uint64_t records{0};
    std::uint64_t rows{0};
};

/// What dump_records() wrote: one file per channel present, in channel order,
/// and the number of records.
struct DumpSummary {
    std::vector<DumpedFile> files;
    std::uint64_t records{0};
};

/// Dump every remaining record of `reader` into `out_dir`, one file per
/// channel, a framework channel in its fixed format and a mission channel in
/// its `mission` format. Returns what was written, or the first error.
/// Pre: out_dir exists; every mission channel the reader accepts has a format.
[[nodiscard]] inline result<DumpSummary, std::string>
dump_records(LogReader& reader, const std::filesystem::path& out_dir, std::span<const ChannelFormat> mission) {
    struct Sink {
        std::string file_name;
        std::filesystem::path path;
        std::ofstream out;
        std::uint64_t records{0};
        std::uint64_t rows{0};
    };
    const auto format_of = [&](Channel c) -> const ChannelFormat* {
        for (const ChannelFormat& f : mission) {
            if (f.channel == c) {
                return &f;
            }
        }
        return nullptr;
    };
    std::map<Channel, Sink> sinks;
    std::uint64_t records = 0;
    for (;;) {
        auto rec = reader.next();
        if (!rec) {
            return rec.error().path + ": " + rec.error().message;
        }
        if (!rec->has_value()) {
            break;
        }
        const LogRecord& r = **rec;
        const ChannelFormat* m = format_of(r.channel);
        const std::string name = m != nullptr ? m->file_name : std::string(framework::file_name(r.channel));
        std::uint64_t n = 0;
        const auto text = m != nullptr ? m->rows(r, n) : framework::rows_of(r, n);
        if (!text) {
            return "undecodable " + name + " record at t=" + std::to_string(r.t.ns);
        }
        auto [it, inserted] = sinks.try_emplace(r.channel);
        Sink& sink = it->second;
        if (inserted) {
            sink.file_name = name;
            sink.path = out_dir / name;
            sink.out.open(sink.path, std::ios::binary | std::ios::trunc);
            if (const std::string header = m != nullptr ? m->columns : framework::columns(r.channel); !header.empty()) {
                sink.out << header << '\n';
            }
        }
        sink.out << *text;
        ++sink.records;
        sink.rows += n;
        ++records;
    }
    DumpSummary summary;
    summary.records = records;
    for (auto& [channel, sink] : sinks) {
        sink.out.close();
        if (sink.out.fail()) {
            return sink.path.string() + ": cannot write the file";
        }
        summary.files.push_back(DumpedFile{channel, sink.file_name, sink.path, sink.records, sink.rows});
    }
    return summary;
}

}  // namespace branes::anatomy::log
