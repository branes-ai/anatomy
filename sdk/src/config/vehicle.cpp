#include <branes/anatomy/config/vehicle.hpp>

#include "parse.hpp"

namespace branes::anatomy::config {

using namespace detail;

namespace {

// ── enum names ──────────────────────────────────────────────────────

CameraModel camera_model(const std::string& s, const std::string& where) {
    if (s == "pinhole_radtan") {
        return CameraModel::pinhole_radtan;
    }
    if (s == "equidistant") {
        return CameraModel::equidistant;
    }
    throw Fail{where, "expected pinhole_radtan or equidistant"};
}
const char* name_of(CameraModel m) {
    return m == CameraModel::pinhole_radtan ? "pinhole_radtan" : "equidistant";
}

Shutter shutter(const std::string& s, const std::string& where) {
    if (s == "global") {
        return Shutter::global;
    }
    if (s == "rolling") {
        return Shutter::rolling;
    }
    throw Fail{where, "expected global or rolling"};
}
const char* name_of(Shutter s) {
    return s == Shutter::global ? "global" : "rolling";
}

PixelFormat pixel_format(const std::string& s, const std::string& where) {
    if (s == "mono8") {
        return PixelFormat::mono8;
    }
    if (s == "rgb8") {
        return PixelFormat::rgb8;
    }
    throw Fail{where, "expected mono8 or rgb8"};
}
const char* name_of(PixelFormat f) {
    return f == PixelFormat::rgb8 ? "rgb8" : "mono8";
}

bool all_positive(const Vec3<double>& v) {
    return v[0] > 0 && v[1] > 0 && v[2] > 0;
}
bool all_non_negative(const Vec3<double>& v) {
    return v[0] >= 0 && v[1] >= 0 && v[2] >= 0;
}

}  // namespace

// ── Vehicle ─────────────────────────────────────────────────────────

result<VehicleConfig, ConfigError> parse_vehicle(std::string_view yaml, std::string_view origin) {
    return guarded<VehicleConfig>(origin, [&]() -> VehicleConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root,
                  "",
                  {"name", "mass_kg", "inertia_kgm2", "arm_length_m", "rotor_count", "rotor", "drag", "imu", "camera"});
        VehicleConfig v;
        v.name = get_string(root, "", "name");
        v.mass = get_double(root, "", "mass_kg");
        require(v.mass > 0, "mass_kg", "must be > 0");
        v.inertia = get_array<3>(root, "", "inertia_kgm2");
        require(all_positive(v.inertia), "inertia_kgm2", "every diagonal entry must be > 0");
        v.arm_length = get_double(root, "", "arm_length_m");
        require(v.arm_length > 0, "arm_length_m", "must be > 0");
        v.rotor_count = static_cast<std::uint32_t>(get_uint(root, "", "rotor_count", 64));
        require(v.rotor_count == 4, "rotor_count", "only quadrotors (4) are supported");

        const YAML::Node rotor = child(root, "", "rotor");
        only_keys(rotor, "rotor", {"thrust_coefficient", "torque_coefficient", "max_thrust_n", "time_constant_s"});
        v.rotor.thrust_coefficient = get_double(rotor, "rotor", "thrust_coefficient");
        v.rotor.torque_coefficient = get_double(rotor, "rotor", "torque_coefficient");
        v.rotor.max_thrust = get_double(rotor, "rotor", "max_thrust_n");
        v.rotor.time_constant = get_double(rotor, "rotor", "time_constant_s");
        require(v.rotor.thrust_coefficient > 0, "rotor.thrust_coefficient", "must be > 0");
        require(v.rotor.torque_coefficient > 0, "rotor.torque_coefficient", "must be > 0");
        require(v.rotor.max_thrust > 0, "rotor.max_thrust_n", "must be > 0");
        require(v.rotor.time_constant > 0, "rotor.time_constant_s", "must be > 0");

        const YAML::Node drag = child(root, "", "drag");
        only_keys(drag, "drag", {"linear", "quadratic"});
        v.drag.linear = get_array<3>(drag, "drag", "linear");
        v.drag.quadratic = get_array<3>(drag, "drag", "quadratic");
        require(all_non_negative(v.drag.linear), "drag.linear", "must be ≥ 0");
        require(all_non_negative(v.drag.quadratic), "drag.quadratic", "must be ≥ 0");

        const YAML::Node imu = child(root, "", "imu");
        only_keys(imu,
                  "imu",
                  {"rate_hz",
                   "gyro_range_rad_s",
                   "accel_range_m_s2",
                   "gyro_noise_density",
                   "accel_noise_density",
                   "gyro_bias_random_walk",
                   "accel_bias_random_walk",
                   "initial_gyro_bias_sigma",
                   "initial_accel_bias_sigma",
                   "latency_s"});
        v.imu.rate_hz = get_double(imu, "imu", "rate_hz");
        v.imu.gyro_range = get_double(imu, "imu", "gyro_range_rad_s");
        v.imu.accel_range = get_double(imu, "imu", "accel_range_m_s2");
        v.imu.gyro_noise_density = get_double(imu, "imu", "gyro_noise_density");
        v.imu.accel_noise_density = get_double(imu, "imu", "accel_noise_density");
        require(checked_period_ns(v.imu.rate_hz).has_value(), "imu.rate_hz", rate_message());
        require(v.imu.gyro_range > 0, "imu.gyro_range_rad_s", "must be > 0");
        require(v.imu.accel_range > 0, "imu.accel_range_m_s2", "must be > 0");
        require(v.imu.gyro_noise_density >= 0, "imu.gyro_noise_density", "must be ≥ 0");
        require(v.imu.accel_noise_density >= 0, "imu.accel_noise_density", "must be ≥ 0");
        v.imu.gyro_bias_random_walk = get_double_or(imu, "imu", "gyro_bias_random_walk", 0);
        v.imu.accel_bias_random_walk = get_double_or(imu, "imu", "accel_bias_random_walk", 0);
        v.imu.initial_gyro_bias_sigma = get_double_or(imu, "imu", "initial_gyro_bias_sigma", 0);
        v.imu.initial_accel_bias_sigma = get_double_or(imu, "imu", "initial_accel_bias_sigma", 0);
        v.imu.latency_s = get_double_or(imu, "imu", "latency_s", 0);
        require(v.imu.gyro_bias_random_walk >= 0, "imu.gyro_bias_random_walk", "must be ≥ 0");
        require(v.imu.accel_bias_random_walk >= 0, "imu.accel_bias_random_walk", "must be ≥ 0");
        require(v.imu.initial_gyro_bias_sigma >= 0, "imu.initial_gyro_bias_sigma", "must be ≥ 0");
        require(v.imu.initial_accel_bias_sigma >= 0, "imu.initial_accel_bias_sigma", "must be ≥ 0");
        require(v.imu.latency_s >= 0 && v.imu.latency_s <= kMaxImuLatencyS,
                "imu.latency_s",
                "must be in [0, " + std::to_string(kMaxImuLatencyS) + "] s");

        const YAML::Node cam = child(root, "", "camera");
        only_keys(cam,
                  "camera",
                  {"model",
                   "width",
                   "height",
                   "format",
                   "fx",
                   "fy",
                   "cx",
                   "cy",
                   "distortion",
                   "rate_hz",
                   "shutter",
                   "row_readout_ns",
                   "translation_body_m",
                   "rotation_body_wxyz"});
        v.camera.model = camera_model(get_string(cam, "camera", "model"), "camera.model");
        v.camera.width = static_cast<std::uint32_t>(get_uint(cam, "camera", "width", 1u << 16));
        v.camera.height = static_cast<std::uint32_t>(get_uint(cam, "camera", "height", 1u << 16));
        v.camera.format = pixel_format(get_string(cam, "camera", "format"), "camera.format");
        v.camera.fx = get_double(cam, "camera", "fx");
        v.camera.fy = get_double(cam, "camera", "fy");
        v.camera.cx = get_double(cam, "camera", "cx");
        v.camera.cy = get_double(cam, "camera", "cy");
        v.camera.distortion = get_array<4>(cam, "camera", "distortion");
        v.camera.rate_hz = get_double(cam, "camera", "rate_hz");
        v.camera.shutter = shutter(get_string(cam, "camera", "shutter"), "camera.shutter");
        v.camera.row_readout_ns = get_int64(cam, "camera", "row_readout_ns");
        v.camera.translation_body = get_array<3>(cam, "camera", "translation_body_m");
        v.camera.rotation_body = get_quat(cam, "camera", "rotation_body_wxyz");
        require(v.camera.width > 0 && v.camera.height > 0, "camera", "width and height must be > 0");
        require(v.camera.fx > 0 && v.camera.fy > 0, "camera", "fx and fy must be > 0");
        require(checked_period_ns(v.camera.rate_hz).has_value(), "camera.rate_hz", rate_message());
        require(v.camera.row_readout_ns >= 0, "camera.row_readout_ns", "must be ≥ 0");
        require(v.camera.shutter == Shutter::rolling || v.camera.row_readout_ns == 0,
                "camera.row_readout_ns",
                "must be 0 for a global shutter");
        return v;
    });
}

result<VehicleConfig, ConfigError> load_vehicle(const std::filesystem::path& path) {
    return load_file<VehicleConfig>(path, parse_vehicle);
}

std::string emit_vehicle(const VehicleConfig& v) {
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << v.name;
    out << YAML::Key << "mass_kg" << YAML::Value << v.mass;
    out << YAML::Key << "inertia_kgm2" << YAML::Value;
    emit_seq(out, v.inertia);
    out << YAML::Key << "arm_length_m" << YAML::Value << v.arm_length;
    out << YAML::Key << "rotor_count" << YAML::Value << v.rotor_count;
    out << YAML::Key << "rotor" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "thrust_coefficient" << YAML::Value << v.rotor.thrust_coefficient;
    out << YAML::Key << "torque_coefficient" << YAML::Value << v.rotor.torque_coefficient;
    out << YAML::Key << "max_thrust_n" << YAML::Value << v.rotor.max_thrust;
    out << YAML::Key << "time_constant_s" << YAML::Value << v.rotor.time_constant;
    out << YAML::EndMap;
    out << YAML::Key << "drag" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "linear" << YAML::Value;
    emit_seq(out, v.drag.linear);
    out << YAML::Key << "quadratic" << YAML::Value;
    emit_seq(out, v.drag.quadratic);
    out << YAML::EndMap;
    out << YAML::Key << "imu" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rate_hz" << YAML::Value << v.imu.rate_hz;
    out << YAML::Key << "gyro_range_rad_s" << YAML::Value << v.imu.gyro_range;
    out << YAML::Key << "accel_range_m_s2" << YAML::Value << v.imu.accel_range;
    out << YAML::Key << "gyro_noise_density" << YAML::Value << v.imu.gyro_noise_density;
    out << YAML::Key << "accel_noise_density" << YAML::Value << v.imu.accel_noise_density;
    out << YAML::Key << "gyro_bias_random_walk" << YAML::Value << v.imu.gyro_bias_random_walk;
    out << YAML::Key << "accel_bias_random_walk" << YAML::Value << v.imu.accel_bias_random_walk;
    out << YAML::Key << "initial_gyro_bias_sigma" << YAML::Value << v.imu.initial_gyro_bias_sigma;
    out << YAML::Key << "initial_accel_bias_sigma" << YAML::Value << v.imu.initial_accel_bias_sigma;
    out << YAML::Key << "latency_s" << YAML::Value << v.imu.latency_s;
    out << YAML::EndMap;
    out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "model" << YAML::Value << name_of(v.camera.model);
    out << YAML::Key << "width" << YAML::Value << v.camera.width;
    out << YAML::Key << "height" << YAML::Value << v.camera.height;
    out << YAML::Key << "format" << YAML::Value << name_of(v.camera.format);
    out << YAML::Key << "fx" << YAML::Value << v.camera.fx;
    out << YAML::Key << "fy" << YAML::Value << v.camera.fy;
    out << YAML::Key << "cx" << YAML::Value << v.camera.cx;
    out << YAML::Key << "cy" << YAML::Value << v.camera.cy;
    out << YAML::Key << "distortion" << YAML::Value;
    emit_seq(out, v.camera.distortion);
    out << YAML::Key << "rate_hz" << YAML::Value << v.camera.rate_hz;
    out << YAML::Key << "shutter" << YAML::Value << name_of(v.camera.shutter);
    out << YAML::Key << "row_readout_ns" << YAML::Value << v.camera.row_readout_ns;
    out << YAML::Key << "translation_body_m" << YAML::Value;
    emit_seq(out, v.camera.translation_body);
    out << YAML::Key << "rotation_body_wxyz" << YAML::Value;
    emit_quat(out, v.camera.rotation_body);
    out << YAML::EndMap;
    out << YAML::EndMap;
    return finish(out);
}

}  // namespace branes::anatomy::config
