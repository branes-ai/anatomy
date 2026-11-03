// log_compare: two replay logs, equal but for the header fields left out
// (log/compare.hpp; docs/plan/flow-graph, 2c).
//
//   log_compare --a <a.brlog> --b <b.brlog> [--except <field>[,<field>...]]
//
// Exits 0 when the logs are equal, 1 when they differ, naming the first
// difference, and 2 when one can't be read or the arguments are wrong. A
// mission's gate compares the logs of a baseline's two builds, its slot
// stack's and its graph's, with --except composition_hash.

#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/log/compare.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

namespace an = branes::anatomy;

namespace {

int usage(const std::string& message) {
    std::fprintf(stderr, "log_compare: %s\n", message.c_str());
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return usage(args.error());
    }
    if (auto bad = args->only({"a", "b", "except"})) {
        return usage(*bad);
    }
    const auto a = args->value("a");
    const auto b = args->value("b");
    if (!a || !b) {
        return usage("usage: --a <a.brlog> --b <b.brlog> [--except <field>[,<field>...]]");
    }
    const std::string except_list = args->value("except").value_or("");
    std::vector<std::string_view> except;
    for (std::size_t at = 0; at < except_list.size();) {
        const std::size_t end = std::min(except_list.find(',', at), except_list.size());
        except.push_back(std::string_view(except_list).substr(at, end - at));
        at = end + 1;
    }

    const auto c = an::log::compare(*a, *b, except);
    if (!c) {
        return usage(c.error().path + ": " + c.error().message);
    }
    if (c->difference) {
        std::printf("log_compare: %s and %s differ: %s\n", a->c_str(), b->c_str(), c->difference->c_str());
        return 1;
    }
    std::printf("log_compare: %s and %s are equal, %llu records%s%s\n",
                a->c_str(),
                b->c_str(),
                static_cast<unsigned long long>(c->records),
                except.empty() ? "" : ", except ",
                except_list.c_str());
    return 0;
}
