#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>

#include "kinematics/arm_4dof.hpp"

using kin::Arm4DofGeometry;
using kin::Arm4DofJoints;
using kin::forward_kinematics_4dof;

// erreur d’arrondi attendue ~1e-16, marge ×10⁴
constexpr double tolerance = 1e-12;

namespace {
void chech_vector_near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
    CHECK(actual.x() == Catch::Approx(expected.x()).margin(tolerance).epsilon(0));
    CHECK(actual.y() == Catch::Approx(expected.y()).margin(tolerance).epsilon(0));
    CHECK(actual.z() == Catch::Approx(expected.z()).margin(tolerance).epsilon(0));
}

Eigen::Vector3d planar_oracle(const Arm4DofGeometry& arm, const double base, const double shoulder,
                              const double elbow, const double wrist) {
    const double radius = arm.upper_arm * std::cos(shoulder) +
                          arm.forearm * std::cos(shoulder + elbow) +
                          arm.hand * std::cos(shoulder + elbow + wrist);
    const double z = arm.shoulder_height + arm.upper_arm * std::sin(shoulder) +
                     arm.forearm * std::sin(shoulder + elbow) +
                     arm.hand * std::sin(shoulder + elbow + wrist);
    return Eigen::Vector3d{radius * std::cos(base), radius * std::sin(base), z};
}
}  // namespace

struct Arm4DofFixture {
    const Arm4DofGeometry arm{
        .shoulder_height = 1.2, .upper_arm = 0.75, .forearm = 0.60, .hand = 0.45};
};

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics of 4-DOF arm straight",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{.base = 0.0, .shoulder = 0.0, .elbow = 0.0, .wrist = 0.0};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected{arm.upper_arm + arm.forearm + arm.hand, 0.0,
                                   arm.shoulder_height};

    chech_vector_near(position, expected);
}

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics base rotation 90 degrees",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{
        .base = std::numbers::pi / 2, .shoulder = 0.0, .elbow = 0.0, .wrist = 0.0};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected{0.0, arm.upper_arm + arm.forearm + arm.hand,
                                   arm.shoulder_height};

    chech_vector_near(position, expected);
}

TEST_CASE_METHOD(Arm4DofFixture, "Base rotation preserves radius and altitude and sets azimuth",
                 "[forward_kinematics_4dof]") {
    const double expected_radius = arm.upper_arm + arm.forearm + arm.hand;
    const std::array<double, 5> bases{0.0, std::numbers::pi / 6, std::numbers::pi / 3,
                                      -std::numbers::pi / 4, std::numbers::pi};

    for (const double base : bases) {
        const Arm4DofJoints q{.base = base, .shoulder = 0.0, .elbow = 0.0, .wrist = 0.0};
        const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);

        const double radius = std::hypot(position.x(), position.y());
        INFO("Base angle: " << base);
        REQUIRE(radius == Catch::Approx(expected_radius).margin(tolerance).epsilon(0));
        REQUIRE(position.z() == Catch::Approx(arm.shoulder_height).margin(tolerance).epsilon(0));
        REQUIRE(std::atan2(position.y(), position.x()) ==
                Catch::Approx(base).margin(tolerance).epsilon(0));
    }
}

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics arm upright straight",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{
        .base = 0.0, .shoulder = std::numbers::pi / 2, .elbow = 0.0, .wrist = 0.0};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected{0.0, 0.0,
                                   arm.shoulder_height + arm.upper_arm + arm.forearm + arm.hand};

    chech_vector_near(position, expected);
}

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics wrist rotation -90 degrees",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{
        .base = 0.0, .shoulder = 0.0, .elbow = 0.0, .wrist = -std::numbers::pi / 2};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected{arm.upper_arm + arm.forearm, 0.0,
                                   arm.shoulder_height - arm.hand};

    chech_vector_near(position, expected);
}

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics elbow rotation 90 degrees",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{
        .base = 0.0, .shoulder = 0.0, .elbow = std::numbers::pi / 2, .wrist = 0.0};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected{arm.upper_arm, 0.0,
                                   arm.shoulder_height + arm.forearm + arm.hand};

    chech_vector_near(position, expected);
}

TEST_CASE_METHOD(Arm4DofFixture, "Planar oracle with several base and pitch combinations",
                 "[forward_kinematics_4dof]") {
    const std::array<double, 4> bases{0.0, std::numbers::pi / 6, std::numbers::pi / 3,
                                      -std::numbers::pi / 4};
    const std::array<double, 3> shoulders{0.0, 0.5, -0.7};
    const std::array<double, 3> elbows{0.0, -0.3, 0.9};
    const std::array<double, 3> wrists{0.0, 0.2, -0.4};

    for (const double base : bases) {
        for (const double shoulder : shoulders) {
            for (const double elbow : elbows) {
                for (const double wrist : wrists) {
                    const Arm4DofJoints q{
                        .base = base, .shoulder = shoulder, .elbow = elbow, .wrist = wrist};
                    const Eigen::Vector3d actual = forward_kinematics_4dof(arm, q);
                    const Eigen::Vector3d expected =
                        planar_oracle(arm, base, shoulder, elbow, wrist);
                    INFO("base=" << base << ", shoulder=" << shoulder << ", elbow=" << elbow
                                 << ", wrist=" << wrist);
                    chech_vector_near(actual, expected);
                }
            }
        }
    }
}

TEST_CASE_METHOD(Arm4DofFixture, "Forward kinematics oracle with non-zero base and pitch angles",
                 "[forward_kinematics_4dof]") {
    const Arm4DofJoints q{.base = std::numbers::pi / 2,
                          .shoulder = 0.5,
                          .elbow = std::numbers::pi / 2,
                          .wrist = -std::numbers::pi / 2};

    const Eigen::Vector3d position = forward_kinematics_4dof(arm, q);
    const Eigen::Vector3d expected = planar_oracle(arm, q.base, q.shoulder, q.elbow, q.wrist);

    chech_vector_near(position, expected);
}