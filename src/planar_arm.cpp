#include "kinematics/planar_arm.hpp"

#include <stdexcept>
#include <cmath>

namespace kin {

Eigen::Matrix3d link_transform(double theta, double length) {
    // TODO(phase0) : construire la matrice homogène 2D décrite dans le header.
    //   - un bloc rotation 2x2 (cos/sin de theta) en haut à gauche,
    //   - la translation de `length` le long de l'axe x LOCAL (après rotation),
    //   - la dernière ligne (0 0 1).
    // Remplace l'identité ci-dessous par la vraie construction.

    const Eigen::Matrix3d matRota ({
        {std::cos(theta), -std::sin(theta), 0},
        {std::sin(theta),  std::cos(theta), 0},
        {0,           0,          1}
    });

    const Eigen::Matrix3d  matTrans ({
        {1, 0, length},
        {0, 1, 0},
        {0, 0, 1}
    });

    return matRota * matTrans;
}

Pose2D forward_kinematics(const PlanarArm& arm, const std::vector<double>& angles) {
    // TODO(phase0) : chaîner les transformations de chaque segment, puis lire la pose.
    //   1. Partir de l'identité (repère de la base).
    //   2. Pour chaque articulation i : T = T * link_transform(angles[i], arm.link_lengths[i]).
    //   3. La position de l'effecteur est la colonne de translation de T.
    //   4. L'orientation theta se lit sur le bloc rotation (atan2).
    // (Réfléchis à la précondition angles.size() == arm.dof() : que faire si elle est violée ?)
    if (angles.size() != arm.dof()) {
        throw std::invalid_argument("angles size does not match arm degrees of freedom");
    }

    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    for (size_t i = 0; i < angles.size(); ++i) {
        T = T * link_transform(angles[i], arm.link_lengths[i]);
    }
    return Pose2D{T(0, 2), T(1, 2), std::atan2(T(1, 0), T(0, 0))};
}

}  // namespace kin
