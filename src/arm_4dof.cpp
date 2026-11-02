#include "kinematics/arm_4dof.hpp"

#include <Eigen/Geometry>

namespace kin {
namespace {
// La convention de rotation autour de l'axe Y est définie par la règle de la main droite.
// Pour obtenir le sens de rotation attendu par le modèle du bras, on applique une rotation
// négative autour de l'axe Y, indépendamment du point de vue de l'observateur.
Eigen::Affine3d pitch_link(double angle, double length) {
    return Eigen::AngleAxisd(angle, -Eigen::Vector3d::UnitY()) *
           Eigen::Translation3d(length, 0.0, 0.0);
}
}  // namespace

Eigen::Vector3d forward_kinematics_4dof(const Arm4DofGeometry& arm, const Arm4DofJoints& q) {
    const Eigen::Affine3d transform_base = Eigen::AngleAxisd(q.base, Eigen::Vector3d::UnitZ()) *
                                           Eigen::Translation3d(0.0, 0.0, arm.shoulder_height);

    const Eigen::Affine3d transform_shoulder = pitch_link(q.shoulder, arm.upper_arm);
    const Eigen::Affine3d transform_elbow = pitch_link(q.elbow, arm.forearm);
    const Eigen::Affine3d transform_wrist = pitch_link(q.wrist, arm.hand);

    const Eigen::Affine3d transform_total =
        transform_base * transform_shoulder * transform_elbow * transform_wrist;

    return transform_total.translation();
}

}  // namespace kin
