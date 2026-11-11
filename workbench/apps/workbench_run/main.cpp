// workbench_run: a pattern of the workbench, run with its default
// configuration, and its oracle's report (docs/plan/workbench).
//
//   workbench_run <mlp|kalman|dsp|pid|attitude> [--inputs N] [--seed S] [--trace run.brtrace]
//                 [--mermaid g.mmd] [--canonical g.txt] [--tolerance-scale K]
//                 [--check] [--compute cd.txt]
//
// --compute writes each computational domain its handlers declare
// (flow/compute.hpp) as text. --check then checks them (K1–K8), one line
// each, instead of running the pattern, and exits 1 if one fails; without
// it, the pattern runs (docs/plan/computational-domain, C2).
//
// Exits 0 if every check passes, 1 if one fails, and 2 on a usage error or a
// file it can't write, as trace_compare does. --tolerance-scale multiplies
// every upper bound by K, to show how much margin the pattern has.
// This tool runs only the patterns compiled into it: the MLP, the Kalman
// filter, the DSP pipeline, PID control and the attitude filter. --inputs
// defaults to the pattern's own number: the Kalman filter's oracle needs 2000
// updates, the DSP pipeline's 32 frames are 8224 inputs, PID control's 15 s
// are 1500 ticks, and the attitude filter's 4 s are 3600 samples.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/attitude.hpp>
#include <branes/anatomy/workbench/patterns/dsp.hpp>
#include <branes/anatomy/workbench/patterns/kalman.hpp>
#include <branes/anatomy/workbench/patterns/mlp.hpp>
#include <branes/anatomy/workbench/patterns/pid.hpp>

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace an = branes::anatomy;
namespace wb = branes::anatomy::workbench;

namespace {

int usage(const std::string& message) {
    std::fprintf(stderr, "workbench_run: %s\n", message.c_str());
    return 2;
}

constexpr const char* kUsage = "usage: workbench_run <mlp|kalman|dsp|pid|attitude> [--inputs N] [--seed S] [--trace "
                               "run.brtrace] [--mermaid g.mmd] "
                               "[--canonical g.txt] [--tolerance-scale K] [--check] [--compute cd.txt]";

template <class T>
std::optional<T> number(const std::string& s) {
    T v{};
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || end != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

bool write_file(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
    return static_cast<bool>(out);
}

template <class P>
concept declares = requires { typename P::leaves; };

/// The pattern's declared handlers, checked: one line each, PASS or FAIL.
template <class P>
int check_declarations() {
    if constexpr (!declares<P>) {
        std::printf("%s: no handler declares a computational domain\n", std::string(P::name).c_str());
        return 0;
    } else {
        std::string lines;
        std::size_t total = 0;
        std::size_t failed = 0;
        wb::for_each_leaf<typename P::leaves>([&]<class Op, an::flow::fixed_string H>() {
            const an::flow::cd::report r = an::flow::cd::check<Op, H>();
            ++total;
            const std::string who = std::string(Op::name) + "." + std::string(H.view());
            if (!r.ok()) {
                ++failed;
                lines += "  FAIL  " + who + "\n";
                for (const auto& p : r.problems) {
                    lines += "        " + p.id + ": " + p.what + "\n";
                }
                return;
            }
            std::string cls;
            for (const auto st : r.stages) {
                cls += (cls.empty() ? "" : "+") + std::string(an::flow::cd::to_string(st));
            }
            std::string ops;
            for (const an::flow::cd::op o :
                 {an::flow::cd::op::mul, an::flow::cd::op::add, an::flow::cd::op::sub, an::flow::cd::op::div}) {
                if (const auto n = r.count(o)) {
                    ops += ", " + std::to_string(n) + " " + std::string(an::flow::cd::to_string(o));
                }
            }
            char line[256];
            std::snprintf(line,
                          sizeof line,
                          "  PASS  %-28s %s, depth %llu%s\n",
                          who.c_str(),
                          cls.c_str(),
                          static_cast<unsigned long long>(r.depth),
                          ops.c_str());
            lines += line;
        });
        std::printf("%s: %zu declarations, %s\n%s",
                    std::string(P::name).c_str(),
                    total,
                    failed ? "some failed" : "all passed",
                    lines.c_str());
        return failed ? 1 : 0;
    }
}

/// Each declared handler's computational domain, as text.
template <class P>
std::string declarations_text() {
    std::string out;
    if constexpr (declares<P>) {
        wb::for_each_leaf<typename P::leaves>([&]<class Op, an::flow::fixed_string H>() {
            out += "# " + std::string(Op::name) + "." + std::string(H.view()) + "\n";
            out += an::flow::cd::text_of<Op, H>() + "\n";
        });
    }
    return out;
}

template <class P>
int run(typename P::config cfg, const an::harness::Args& args, std::optional<std::size_t> inputs, double scale) {
    if (const auto file = args.value("compute"); file && !write_file(*file, declarations_text<P>())) {
        return usage("cannot write " + *file);
    }
    if (args.has("check")) {
        return check_declarations<P>();
    }
    if (const auto mmd = args.value("mermaid"), txt = args.value("canonical"); mmd || txt) {
        const auto g = wb::describe<P>(cfg);
        if (!g) {
            return usage(g.error().to_string());
        }
        if (mmd && !write_file(*mmd, an::flow::mermaid(*g))) {
            return usage("cannot write " + *mmd);
        }
        if (txt && !write_file(*txt, an::flow::canonical(*g))) {
            return usage("cannot write " + *txt);
        }
    }
    wb::run_options opt;
    opt.inputs = inputs ? *inputs : P::default_inputs;
    if (const auto t = args.value("trace")) {
        opt.trace = *t;
    }
    const auto samples = wb::run<P>(cfg, opt);
    if (!samples) {
        return usage(samples.error());
    }
    const wb::report r = wb::scaled(P::oracle(cfg, *samples), scale);
    std::fputs(r.to_string().c_str(), stdout);
    return r.passed() ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || std::string_view(argv[1]).starts_with("--")) {
        return usage(kUsage);
    }
    const std::string pattern = argv[1];
    auto args = an::harness::Args::parse(argc - 1, argv + 1, {"check"});
    if (!args) {
        return usage(args.error());
    }
    if (auto bad =
            args->only({"inputs", "seed", "trace", "mermaid", "canonical", "tolerance-scale", "check", "compute"})) {
        return usage(*bad);
    }
    std::optional<std::size_t> inputs;
    if (const auto n = args->value("inputs")) {
        const auto v = number<std::size_t>(*n);
        if (!v || *v == 0) {
            return usage("--inputs takes a positive integer, not '" + *n + "'");
        }
        inputs = *v;
    }
    std::optional<std::uint64_t> seed;
    if (const auto s = args->value("seed")) {
        seed = number<std::uint64_t>(*s);
        if (!seed) {
            return usage("--seed takes an unsigned integer, not '" + *s + "'");
        }
    }
    double scale = 1.0;
    if (const auto k = args->value("tolerance-scale")) {
        const auto v = number<double>(*k);
        if (!v || !(*v >= 0)) {
            return usage("--tolerance-scale takes a number of at least 0, not '" + *k + "'");
        }
        scale = *v;
    }

    if (pattern == "mlp") {
        wb::mlp::config cfg;
        if (seed) {
            cfg.seed = *seed;
        }
        return run<wb::mlp::pattern>(cfg, *args, inputs, scale);
    }
    if (pattern == "kalman") {
        wb::kalman::config cfg;
        if (seed) {
            cfg.seed = *seed;
        }
        return run<wb::kalman::pattern>(cfg, *args, inputs, scale);
    }
    if (pattern == "dsp") {
        wb::dsp::config cfg;
        if (seed) {
            cfg.seed = *seed;
        }
        return run<wb::dsp::pattern>(cfg, *args, inputs, scale);
    }
    if (pattern == "pid") {
        wb::pid::config cfg;
        if (seed) {
            cfg.seed = *seed;
        }
        return run<wb::pid::pattern>(cfg, *args, inputs, scale);
    }
    if (pattern == "attitude") {
        wb::attitude::config cfg;
        if (seed) {
            cfg.seed = *seed;
        }
        return run<wb::attitude::pattern>(cfg, *args, inputs, scale);
    }
    return usage("no pattern '" + pattern + "': this build runs mlp, kalman, dsp, pid and attitude\n" + kUsage);
}
