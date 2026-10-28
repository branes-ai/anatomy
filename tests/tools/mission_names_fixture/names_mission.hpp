#pragma once
// A fixture for framework.detects_mission_names: code that names a mission's
// types, which the check must find (tests/tools/ is skipped by the real scan).
#include <branes/racing/msg/course.hpp>
inline constexpr int kGates = sizeof(branes::racing::CourseMap<double>);
