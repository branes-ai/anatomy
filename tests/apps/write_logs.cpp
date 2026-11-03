// write_logs <dir>: the logs log_compare_test.cmake compares. a.brlog; hash.brlog,
// a with another composition_hash; payload.brlog, a with another last payload.

#include <branes/anatomy/log/log.hpp>

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace an = branes::anatomy;

namespace {

bool write(const std::filesystem::path& p, const std::string& hash, std::byte last) {
    an::log::LogHeader h;
    h.baseline_name = "null";
    h.composition_hash = hash;
    auto w = an::log::LogWriter::open(p, h);
    if (!w) {
        return false;
    }
    const std::vector<std::byte> first = {std::byte{1}};
    const std::vector<std::byte> second = {last};
    w->write(an::log::Channel::imu, an::Timestamp{10}, first);
    w->write(an::log::Channel::command, an::Timestamp{10}, second);
    return static_cast<bool>(w->close());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: write_logs <dir>\n");
        return 2;
    }
    const std::filesystem::path dir = argv[1];
    if (!write(dir / "a.brlog", "slots", std::byte{2}) || !write(dir / "hash.brlog", "graph", std::byte{2}) ||
        !write(dir / "payload.brlog", "slots", std::byte{3})) {
        std::fprintf(stderr, "write_logs: cannot write the logs in %s\n", dir.string().c_str());
        return 1;
    }
    return 0;
}
