#pragma once

#include <regex>
#include <optional>

/// Finds the appropriate steering angle for some turning radius, which is defined by the
/// transient velocity of the vehicle along that curve (just the current velocity in this instance),
/// and the rate that curve changes (angular z rotation). This angle is effected by vehicle wheelbase.
///
/// All units are in m/s or r/s.
constexpr double convert_trans_rot_vel_to_steering_angle(double vel, double omega, double wheelbase) {
    if (omega == 0 || vel == 0) {
        return 0;
    }

    // Remove negative so steering doesn't reverse when reversing.
    vel = std::abs(vel);

    auto rad = vel / omega;
    return std::atan(wheelbase / rad);
}

/// Parses the collision box as [[f32, f32], [f32, f32]] using regex. Returns None on error.
auto parse_collision_box(
        const std::string &str) -> std::optional<std::tuple<std::tuple<float, float>, std::tuple<float, float>>> {
    // Oh boy, matches [[f32, f32], [f32, f32]] ignoring whitespaces
    static std::regex coll_reg{
            R"(\[\s*\[\s*([+|-]?\d*\.?\d*)\s*,\s*([+|-]?\d*\.?\d*)\s*\]\s*,\s*\[\s*([+|-]?\d*\.?\d*)\s*,\s*([+|-]?\d*\.?\d*)\s*\]\s*\])",
            std::regex_constants::icase};

    std::smatch res{};
    auto tuple1 = std::make_tuple(0.0, 0.0);
    auto tuple2 = std::make_tuple(0.0, 0.0);

    if (std::regex_match(str, res, coll_reg) && res.size() > 4) {
        std::get<0>(tuple1) = std::stod(res[1]);
        std::get<1>(tuple1) = std::stod(res[2]);

        std::get<0>(tuple2) = std::stod(res[3]);
        std::get<1>(tuple2) = std::stod(res[4]);

        return std::make_tuple(tuple1, tuple2);
    } else {
        return std::nullopt;
    }
}

constexpr double rad_to_deg(double val) {
    return val / M_PI * 180;
}

/// Converts lidarscan readings angle into a standard polar frame.
double lidarscan_polar(double reading) {
    auto p_deg = 90 - reading;
    if (p_deg < 0.0) {
        while (p_deg < 0.0) {
            p_deg += 360;
        }
    }

    return p_deg;
}