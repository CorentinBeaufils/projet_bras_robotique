#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "kinematics/planar_arm.hpp"

using Catch::Approx;
using kin::forward_kinematics;
using kin::PlanarArm;
using kin::Pose2D;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST_CASE("FK : un segment aligné sur x", "[fk]") {
    const PlanarArm arm{{1.0}};
    const Pose2D p = forward_kinematics(arm, {0.0});
    CHECK(p.x == Approx(1.0));
    CHECK(p.y == Approx(0.0).margin(1e-9));
    CHECK(p.theta == Approx(0.0).margin(1e-9));
}

TEST_CASE("FK : un segment à 90 degrés", "[fk]") {
    const PlanarArm arm{{1.0}};
    const Pose2D p = forward_kinematics(arm, {kPi / 2.0});
    CHECK(p.x == Approx(0.0).margin(1e-9));
    CHECK(p.y == Approx(1.0));
    CHECK(p.theta == Approx(kPi / 2.0));
}

TEST_CASE("FK : deux segments tendus", "[fk]") {
    const PlanarArm arm{{1.0, 1.0}};
    const Pose2D p = forward_kinematics(arm, {0.0, 0.0});
    CHECK(p.x == Approx(2.0));
    CHECK(p.y == Approx(0.0).margin(1e-9));
    CHECK(p.theta == Approx(0.0).margin(1e-9));
}

TEST_CASE("FK : deux segments, coude à 90 degrés", "[fk]") {
    const PlanarArm arm{{1.0, 1.0}};
    const Pose2D p = forward_kinematics(arm, {0.0, kPi / 2.0});
    CHECK(p.x == Approx(1.0));
    CHECK(p.y == Approx(1.0));
    CHECK(p.theta == Approx(kPi / 2.0));
}

TEST_CASE("FK : trois segments (3 DOF)", "[fk]") {
    const PlanarArm arm{{1.0, 1.0, 1.0}};
    const Pose2D p = forward_kinematics(arm, {kPi / 2.0, -kPi / 2.0, 0.0});
    CHECK(p.x == Approx(2.0));
    CHECK(p.y == Approx(1.0));
    CHECK(p.theta == Approx(0.0).margin(1e-9));
}
