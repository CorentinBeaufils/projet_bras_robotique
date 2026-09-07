#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "kinematics/inverse_kinematics.hpp"
#include "kinematics/planar_arm.hpp"

using Catch::Approx;
using kin::Elbow;
using kin::forward_kinematics;
using kin::inverse_kinematics_2link;
using kin::PlanarArm;
using kin::Pose2D;

namespace {
// Test par ALLER-RETOUR : l'IK a plusieurs solutions, donc on ne fige pas des angles.
// On vérifie que la FK des angles trouvés retombe bien sur la cible.
void check_reaches(double l1, double l2, double x, double y, Elbow e) {
    const auto sol = inverse_kinematics_2link(l1, l2, x, y, e);
    REQUIRE(sol.has_value());
    const PlanarArm arm{{l1, l2}};
    const Pose2D p = forward_kinematics(arm, {sol->theta1, sol->theta2});
    CHECK(p.x == Approx(x).margin(1e-9));
    CHECK(p.y == Approx(y).margin(1e-9));
}
}  // namespace

TEST_CASE("IK : cible atteignable, les deux coudes y arrivent", "[ik]") {
    check_reaches(1.0, 1.0, 1.5, 0.5, Elbow::Up);
    check_reaches(1.0, 1.0, 1.5, 0.5, Elbow::Down);
}

TEST_CASE("IK : segments inégaux", "[ik]") {
    check_reaches(2.0, 1.0, 1.0, 1.5, Elbow::Up);
    check_reaches(2.0, 1.0, 1.0, 1.5, Elbow::Down);
}

TEST_CASE("IK : coude haut et coude bas ont des angles de coude opposés", "[ik]") {
    const auto up = inverse_kinematics_2link(1.0, 1.0, 1.5, 0.5, Elbow::Up);
    const auto down = inverse_kinematics_2link(1.0, 1.0, 1.5, 0.5, Elbow::Down);
    REQUIRE(up.has_value());
    REQUIRE(down.has_value());
    CHECK(up->theta2 == Approx(-down->theta2));
}

TEST_CASE("IK : cible trop loin -> nullopt", "[ik]") {
    CHECK_FALSE(inverse_kinematics_2link(1.0, 1.0, 3.0, 0.0, Elbow::Up).has_value());
}

TEST_CASE("IK : cible dans le trou intérieur -> nullopt", "[ik]") {
    // l1=2, l2=1 -> rayon intérieur 1 ; un point à 0.5 de la base est inatteignable.
    CHECK_FALSE(inverse_kinematics_2link(2.0, 1.0, 0.5, 0.0, Elbow::Up).has_value());
}

TEST_CASE("IK : bras tendu à la portée maximale", "[ik]") {
    check_reaches(1.0, 1.0, 2.0, 0.0, Elbow::Up);
}
