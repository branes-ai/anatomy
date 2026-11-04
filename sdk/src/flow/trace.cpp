#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <array>
#include <cassert>
#include <span>

namespace branes::anatomy::flow {

namespace {

constexpr std::array<char, 8> kMagic = {'B', 'R', 'T', 'R', 'A', 'C', 'E', '\0'};

void write_bytes(std::ofstream& out, std::span<const std::byte> b) {
    out.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

bool read_bytes(std::ifstream& in, std::span<std::byte> b) {
    in.read(reinterpret_cast<char*>(b.data()), static_cast<std::streamsize>(b.size()));
    return static_cast<std::size_t>(in.gcount()) == b.size();
}

std::vector<std::byte> encode_record(const trace_record& r) {
    ByteWriter w;
    w.str(r.region);
    w.u32(r.firing);
    w.str(r.op);
    w.str(r.handler);
    w.str(r.composite);
    w.str(r.port);
    w.i64(r.t.ns);
    w.boolean(r.payload.has_value());
    if (r.payload) {
        w.u32(static_cast<std::uint32_t>(r.payload->size()));
        w.bytes(*r.payload);
    }
    return w.take();
}

std::optional<trace_record> decode_record(std::span<const std::byte> b) {
    ByteReader r(b);
    trace_record rec;
    rec.region = r.str();
    rec.firing = r.u32();
    rec.op = r.str();
    rec.handler = r.str();
    rec.composite = r.str();
    rec.port = r.str();
    rec.t = Timestamp{r.i64()};
    if (r.boolean()) {
        const std::uint32_t n = r.u32();
        const auto p = r.bytes(n);
        rec.payload = std::vector<std::byte>(p.begin(), p.end());
    }
    if (!r.done()) {
        return std::nullopt;
    }
    return rec;
}

}  // namespace

// ── trace_writer ────────────────────────────────────────────────────

trace_writer::trace_writer(std::ofstream out, std::filesystem::path path, Graph g, trace_codecs codecs)
    : out_(std::move(out)), path_(std::move(path)), graph_(std::move(g)), codecs_(std::move(codecs)), open_(true) {}

trace_writer::trace_writer(trace_writer&& other) noexcept
    : out_(std::move(other.out_)), path_(std::move(other.path_)), graph_(std::move(other.graph_)),
      codecs_(std::move(other.codecs_)), open_(std::exchange(other.open_, false)), records_(other.records_) {}

trace_writer& trace_writer::operator=(trace_writer&& other) noexcept {
    if (this != &other) {
        if (open_) {
            (void)close();
        }
        out_ = std::move(other.out_);
        path_ = std::move(other.path_);
        graph_ = std::move(other.graph_);
        codecs_ = std::move(other.codecs_);
        open_ = std::exchange(other.open_, false);
        records_ = other.records_;
    }
    return *this;
}

trace_writer::~trace_writer() {
    if (open_) {
        (void)close();
    }
}

result<trace_writer, log::LogError>
trace_writer::open(const std::filesystem::path& path, const Graph& g, trace_codecs codecs) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return log::LogError{path.string(), "cannot create the trace file"};
    }
    ByteWriter h;
    h.str(g.mission);
    h.str(graph_hash(g));
    ByteWriter prefix;
    for (const char c : kMagic) {
        prefix.u8(static_cast<std::uint8_t>(c));
    }
    prefix.u32(kTraceVersion);
    prefix.u32(static_cast<std::uint32_t>(h.data().size()));
    write_bytes(out, prefix.data());
    write_bytes(out, h.data());
    if (!out) {
        return log::LogError{path.string(), "cannot write the trace header"};
    }
    return trace_writer(std::move(out), path, g, std::move(codecs));
}

void trace_writer::write(const fired& f) {
    assert(open_);
    const Region& region = graph_.regions.at(f.region);
    const Firing& firing = region.firings.at(f.firing);
    for (std::size_t k = 0; k < f.ports.size(); ++k) {
        trace_record rec{region.trigger,
                         static_cast<std::uint32_t>(f.firing),
                         firing.op,
                         firing.handler,
                         firing.composite,
                         f.ports[k],
                         f.t,
                         std::nullopt};
        if (const auto* encode = codecs_.find(f.types[k])) {
            rec.payload = (*encode)(f.messages[k]);
        }
        const std::vector<std::byte> body = encode_record(rec);
        ByteWriter size;
        size.u32(static_cast<std::uint32_t>(body.size()));
        write_bytes(out_, size.data());
        write_bytes(out_, body);
        ++records_;
    }
}

result<ok_t, log::LogError> trace_writer::close() {
    if (!open_) {
        return ok;
    }
    open_ = false;
    out_.flush();
    const bool good = static_cast<bool>(out_);
    out_.close();
    if (!good || out_.fail()) {
        return log::LogError{path_.string(), "write error"};
    }
    return ok;
}

// ── trace_reader ────────────────────────────────────────────────────

result<trace_reader, log::LogError> trace_reader::open(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return log::LogError{path.string(), "cannot open the trace file"};
    }
    std::array<std::byte, 16> prefix{};
    if (!read_bytes(in, prefix)) {
        return log::LogError{path.string(), "file too short for a trace"};
    }
    for (std::size_t i = 0; i < kMagic.size(); ++i) {
        if (prefix[i] != static_cast<std::byte>(kMagic[i])) {
            return log::LogError{path.string(), "not a trace (bad magic)"};
        }
    }
    ByteReader r(std::span<const std::byte>(prefix).subspan(8));
    const std::uint32_t version = r.u32();
    const std::uint32_t size = r.u32();
    if (version != kTraceVersion) {
        return log::LogError{path.string(), "unsupported trace version " + std::to_string(version)};
    }
    if (size > log::kMaxPayload) {
        return log::LogError{path.string(), "corrupt header size"};
    }
    std::vector<std::byte> h(size);
    if (!read_bytes(in, h)) {
        return log::LogError{path.string(), "truncated header"};
    }
    ByteReader hr(h);
    trace_header header;
    header.mission = hr.str();
    header.graph_hash = hr.str();
    if (!hr.done()) {
        return log::LogError{path.string(), "corrupt header"};
    }
    return trace_reader(std::move(in), path, std::move(header));
}

result<std::optional<trace_record>, log::LogError> trace_reader::next() {
    std::array<std::byte, 4> size_bytes{};
    in_.read(reinterpret_cast<char*>(size_bytes.data()), static_cast<std::streamsize>(size_bytes.size()));
    const auto got = static_cast<std::size_t>(in_.gcount());
    if (got == 0 && in_.eof()) {
        return std::optional<trace_record>{};
    }
    if (got != size_bytes.size()) {
        return log::LogError{path_.string(), "truncated record size"};
    }
    ByteReader sr(size_bytes);
    const std::uint32_t size = sr.u32();
    if (size > log::kMaxPayload) {
        return log::LogError{path_.string(), "corrupt record size"};
    }
    std::vector<std::byte> body(size);
    if (!read_bytes(in_, body)) {
        return log::LogError{path_.string(), "truncated record"};
    }
    auto rec = decode_record(body);
    if (!rec) {
        return log::LogError{path_.string(), "corrupt record"};
    }
    return std::optional<trace_record>{std::move(*rec)};
}

// ── compare_traces ──────────────────────────────────────────────────

namespace {

std::string named(const trace_record& r) {
    const std::string at = r.composite.empty() ? "" : ", in " + r.composite;
    return r.edge() + " (" + r.op + "." + r.handler + at + ")";
}

/// How two records differ, or nullopt.
std::optional<std::string> differs(const trace_record& a, const trace_record& b) {
    if (a.region != b.region || a.firing != b.firing || a.port != b.port || a.op != b.op || a.handler != b.handler ||
        a.composite != b.composite) {
        return "a's is " + named(a) + ", b's " + named(b);
    }
    if (a.t != b.t) {
        return named(a) + " is at t=" + std::to_string(a.t.ns) + " ns in a, at t=" + std::to_string(b.t.ns) +
               " ns in b";
    }
    if (a.payload.has_value() != b.payload.has_value()) {
        return named(a) + " has its value in " + (a.payload ? "a" : "b") + " only";
    }
    if (a.payload != b.payload) {
        return named(a) + " at t=" + std::to_string(a.t.ns) + " ns has another value";
    }
    return std::nullopt;
}

}  // namespace

result<log::Comparison, log::LogError> compare_traces(const std::filesystem::path& a, const std::filesystem::path& b) {
    auto ra = trace_reader::open(a);
    if (!ra) {
        return ra.error();
    }
    auto rb = trace_reader::open(b);
    if (!rb) {
        return rb.error();
    }
    log::Comparison c;
    for (;;) {
        auto x = ra->next();
        if (!x) {
            return x.error();
        }
        auto y = rb->next();
        if (!y) {
            return y.error();
        }
        if (!*x && !*y) {
            return c;
        }
        const std::string at = "record " + std::to_string(c.records);
        if (!*x || !*y) {
            const trace_record& more = *x ? **x : **y;
            c.difference = at + ": " + std::string(*x ? "a" : "b") + " has more records, from " + named(more);
            return c;
        }
        if (const auto d = differs(**x, **y)) {
            c.difference = at + ": " + *d;
            return c;
        }
        ++c.records;
    }
}

}  // namespace branes::anatomy::flow
