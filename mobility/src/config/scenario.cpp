#include <branes/anatomy/config/yaml.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>

#include <array>
#include <cassert>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace branes::anatomy::config {

using namespace yaml;

// ── Scenario ────────────────────────────────────────────────────────

namespace {

/// "must be ≥ <min>", with the bound in the shortest form that reads back exactly.
std::string at_least(double min) {
    std::ostringstream ss;
    ss.precision(17);
    ss << min;
    return "must be ≥ " + ss.str();
}

}  // namespace

result<ScenarioConfig, ConfigError> parse_scenario(std::string_view yaml,
                                                   std::string_view origin,
                                                   const std::filesystem::path& base_dir,
                                                   const ScenarioSchema& schema) {
    return guarded<ScenarioConfig>(origin, [&]() -> ScenarioConfig {
        const YAML::Node root = parse_root(yaml);
        const std::array<std::string_view, 9> root_keys = {"name",
                                                           schema.world_key,
                                                           "vehicle",
                                                           "duration_s",
                                                           "seed",
                                                           "speed_limit_m_s",
                                                           "sim",
                                                           "randomization",
                                                           "faults"};
        only_keys(root, "", root_keys);
        const std::string world_key(schema.world_key);
        ScenarioConfig s;
        s.name = get_string(root, "", "name");
        s.world_ref = get_string(root, "", world_key);
        s.vehicle_ref = get_string(root, "", "vehicle");
        s.world_path = (base_dir / s.world_ref).lexically_normal();
        s.vehicle_path = (base_dir / s.vehicle_ref).lexically_normal();
        std::error_code ec;
        require(std::filesystem::is_regular_file(s.world_path, ec), world_key, "no file " + s.world_path.string());
        require(std::filesystem::is_regular_file(s.vehicle_path, ec), "vehicle", "no file " + s.vehicle_path.string());
        s.duration_s = get_double(root, "", "duration_s");
        require(s.duration_s > 0 && s.duration_s <= kMaxDurationS,
                "duration_s",
                "must be in (0, " + std::to_string(kMaxDurationS) + "] s");
        s.seed = get_uint(root, "", "seed", std::numeric_limits<std::uint64_t>::max());
        if (root["speed_limit_m_s"]) {
            const double v = get_double(root, "", "speed_limit_m_s");
            require(v > 0, "speed_limit_m_s", "must be > 0");
            s.speed_limit_m_s = v;
        }

        s.world_sim = default_world_sim(schema);
        if (const YAML::Node sim = root["sim"]) {
            std::vector<std::string_view> sim_keys = {
                "physics_rate_hz", "command_timeout_s", "fc_gyro_lowpass_hz", "test_pattern_frames"};
            for (const WorldSimParam& w : schema.world_sim) {
                sim_keys.push_back(w.key);
            }
            only_keys(sim, "sim", sim_keys);
            SimParams& p = s.sim;
            p.physics_rate_hz = get_double_or(sim, "sim", "physics_rate_hz", p.physics_rate_hz);
            p.command_timeout_s = get_double_or(sim, "sim", "command_timeout_s", p.command_timeout_s);
            p.fc_gyro_lowpass_hz = get_double_or(sim, "sim", "fc_gyro_lowpass_hz", p.fc_gyro_lowpass_hz);
            for (std::size_t i = 0; i < schema.world_sim.size(); ++i) {
                s.world_sim[i] = get_double_or(sim, "sim", std::string(schema.world_sim[i].key), s.world_sim[i]);
            }
            if (const YAML::Node b = sim["test_pattern_frames"]) {
                require(b.IsScalar() && (b.as<std::string>() == "true" || b.as<std::string>() == "false"),
                        "sim.test_pattern_frames",
                        "expected true or false");
                p.test_pattern_frames = b.as<std::string>() == "true";
            }
            // The plant (reflex QuadrotorModel) needs dt ≤ 10 ms.
            require(checked_period_ns(p.physics_rate_hz).has_value() && p.physics_rate_hz >= 100,
                    "sim.physics_rate_hz",
                    "must be in [100, " + std::to_string(kMaxRateHz) + "] Hz");
            require(
                p.command_timeout_s > 0 && p.command_timeout_s <= 10, "sim.command_timeout_s", "must be in (0, 10] s");
            require(p.fc_gyro_lowpass_hz > 0 && 2 * p.fc_gyro_lowpass_hz < p.physics_rate_hz,
                    "sim.fc_gyro_lowpass_hz",
                    "must be in (0, physics_rate_hz / 2)");
            for (std::size_t i = 0; i < schema.world_sim.size(); ++i) {
                const WorldSimParam& w = schema.world_sim[i];
                require(s.world_sim[i] >= w.min, "sim." + std::string(w.key), at_least(w.min));
            }
        }

        if (const YAML::Node r = root["randomization"]) {
            only_keys(
                r, "randomization", {"mass", "inertia", "drag", "motor_time_constant", "start_xy_m", "start_yaw_rad"});
            Randomization& z = s.randomization;
            const auto fraction = [&](const std::string& key) {
                const double f = get_double_or(r, "randomization", key, 0);
                require(f >= 0 && f < 1, "randomization." + key, "must be in [0, 1)");
                return f;
            };
            z.mass = fraction("mass");
            z.inertia = fraction("inertia");
            z.drag = fraction("drag");
            z.motor_time_constant = fraction("motor_time_constant");
            z.start_xy_m = get_double_or(r, "randomization", "start_xy_m", 0);
            z.start_yaw_rad = get_double_or(r, "randomization", "start_yaw_rad", 0);
            require(z.start_xy_m >= 0, "randomization.start_xy_m", "must be ≥ 0");
            require(z.start_yaw_rad >= 0 && z.start_yaw_rad <= 3.141592653589793,
                    "randomization.start_yaw_rad",
                    "must be in [0, π]");
        }

        if (const YAML::Node faults = root["faults"]) {
            require(faults.IsSequence(), "faults", "expected a list");
            for (std::size_t i = 0; i < faults.size(); ++i) {
                const YAML::Node f = faults[i];
                const std::string p = "faults[" + std::to_string(i) + "]";
                const std::string type = get_string(f, p, "type");
                FaultEvent e;
                bool known = false;
                for (const FaultType t : {FaultType::imu_bias_step,
                                          FaultType::camera_corrupt,
                                          FaultType::camera_blackout,
                                          FaultType::command_delay,
                                          FaultType::link_loss,
                                          FaultType::motor_scale}) {
                    if (type == name_of(t)) {
                        e.type = t;
                        known = true;
                    }
                }
                require(known, p + ".type", "unknown fault type '" + type + "'");
                switch (e.type) {
                case FaultType::imu_bias_step:
                    only_keys(f, p, {"type", "t_s", "gyro", "accel"});
                    break;
                case FaultType::camera_corrupt:
                    only_keys(f, p, {"type", "t_s", "until_s", "probability"});
                    break;
                case FaultType::command_delay:
                    only_keys(f, p, {"type", "t_s", "until_s", "delay_s"});
                    break;
                case FaultType::motor_scale:
                    only_keys(f, p, {"type", "t_s", "until_s", "rotor", "scale"});
                    break;
                case FaultType::camera_blackout:
                case FaultType::link_loss:
                    only_keys(f, p, {"type", "t_s", "until_s"});
                    break;
                }
                e.t_s = get_double(f, p, "t_s");
                e.until_s = get_double_or(f, p, "until_s", kMaxDurationS);
                require(e.t_s >= 0 && e.t_s < e.until_s && e.until_s <= kMaxDurationS,
                        p,
                        "needs 0 ≤ t_s < until_s ≤ " + std::to_string(kMaxDurationS) + " s");
                if (e.type == FaultType::imu_bias_step) {
                    e.gyro = f["gyro"] ? get_array<3>(f, p, "gyro") : Vec3<double>{};
                    e.accel = f["accel"] ? get_array<3>(f, p, "accel") : Vec3<double>{};
                }
                if (e.type == FaultType::camera_corrupt) {
                    e.probability = get_double(f, p, "probability");
                    require(e.probability >= 0 && e.probability <= 1, p + ".probability", "must be in [0, 1]");
                }
                if (e.type == FaultType::command_delay) {
                    e.delay_s = get_double(f, p, "delay_s");
                    require(e.delay_s >= 0 && e.delay_s <= 1, p + ".delay_s", "must be in [0, 1] s");
                }
                if (e.type == FaultType::motor_scale) {
                    e.rotor = static_cast<std::uint32_t>(get_uint(f, p, "rotor", 3));
                    e.scale = get_double(f, p, "scale");
                    require(e.scale >= 0 && e.scale <= 1, p + ".scale", "must be in [0, 1]");
                }
                s.faults.push_back(e);
            }
        }
        return s;
    });
}

result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path, const ScenarioSchema& schema) {
    const auto base = path.parent_path();
    return load_file<ScenarioConfig>(path, [&](std::string_view text, std::string_view origin) {
        return parse_scenario(text, origin, base, schema);
    });
}

std::string emit_scenario(const ScenarioConfig& s, const ScenarioSchema& schema) {
    assert(s.world_sim.size() == schema.world_sim.size());  // Pre
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << s.name;
    out << YAML::Key << std::string(schema.world_key) << YAML::Value << s.world_ref;
    out << YAML::Key << "vehicle" << YAML::Value << s.vehicle_ref;
    out << YAML::Key << "duration_s" << YAML::Value << s.duration_s;
    out << YAML::Key << "seed" << YAML::Value << s.seed;
    if (s.speed_limit_m_s) {
        out << YAML::Key << "speed_limit_m_s" << YAML::Value << *s.speed_limit_m_s;
    }
    out << YAML::Key << "sim" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "physics_rate_hz" << YAML::Value << s.sim.physics_rate_hz;
    out << YAML::Key << "command_timeout_s" << YAML::Value << s.sim.command_timeout_s;
    out << YAML::Key << "fc_gyro_lowpass_hz" << YAML::Value << s.sim.fc_gyro_lowpass_hz;
    for (std::size_t i = 0; i < schema.world_sim.size(); ++i) {
        out << YAML::Key << std::string(schema.world_sim[i].key) << YAML::Value << s.world_sim[i];
    }
    out << YAML::Key << "test_pattern_frames" << YAML::Value << s.sim.test_pattern_frames;
    out << YAML::EndMap;
    out << YAML::Key << "randomization" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "mass" << YAML::Value << s.randomization.mass;
    out << YAML::Key << "inertia" << YAML::Value << s.randomization.inertia;
    out << YAML::Key << "drag" << YAML::Value << s.randomization.drag;
    out << YAML::Key << "motor_time_constant" << YAML::Value << s.randomization.motor_time_constant;
    out << YAML::Key << "start_xy_m" << YAML::Value << s.randomization.start_xy_m;
    out << YAML::Key << "start_yaw_rad" << YAML::Value << s.randomization.start_yaw_rad;
    out << YAML::EndMap;
    out << YAML::Key << "faults" << YAML::Value << YAML::BeginSeq;
    for (const FaultEvent& e : s.faults) {
        out << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << std::string(name_of(e.type));
        out << YAML::Key << "t_s" << YAML::Value << e.t_s;
        if (e.type != FaultType::imu_bias_step) {
            out << YAML::Key << "until_s" << YAML::Value << e.until_s;
        }
        switch (e.type) {
        case FaultType::imu_bias_step:
            out << YAML::Key << "gyro" << YAML::Value;
            emit_seq(out, e.gyro);
            out << YAML::Key << "accel" << YAML::Value;
            emit_seq(out, e.accel);
            break;
        case FaultType::camera_corrupt:
            out << YAML::Key << "probability" << YAML::Value << e.probability;
            break;
        case FaultType::command_delay:
            out << YAML::Key << "delay_s" << YAML::Value << e.delay_s;
            break;
        case FaultType::motor_scale:
            out << YAML::Key << "rotor" << YAML::Value << e.rotor;
            out << YAML::Key << "scale" << YAML::Value << e.scale;
            break;
        case FaultType::camera_blackout:
        case FaultType::link_loss:
            break;
        }
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return finish(out);
}

bool fault_event_valid(const FaultEvent& f) noexcept {
    const auto finite3 = [](const Vec3<double>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };
    const auto unit = [](double x) { return x >= 0 && x <= 1; };  // false for NaN
    return f.t_s >= 0 && f.t_s < f.until_s && f.until_s <= kMaxDurationS && finite3(f.gyro) && finite3(f.accel) &&
           unit(f.probability) && unit(f.delay_s) && unit(f.scale) && f.rotor < 4;
}

std::string_view name_of(FaultType t) noexcept {
    switch (t) {
    case FaultType::imu_bias_step:
        return "imu_bias_step";
    case FaultType::camera_corrupt:
        return "camera_corrupt";
    case FaultType::camera_blackout:
        return "camera_blackout";
    case FaultType::command_delay:
        return "command_delay";
    case FaultType::link_loss:
        return "link_loss";
    case FaultType::motor_scale:
        return "motor_scale";
    }
    return "unknown";
}

}  // namespace branes::anatomy::config
