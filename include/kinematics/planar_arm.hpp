#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace kin {

/// Pose planaire de l'effecteur.
/// x, y : position en mètres dans le repère de la base.
/// theta : orientation de l'effecteur en radians (somme des angles articulaires).
struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// Bras plan à chaîne ouverte, articulations rotoïdes.
/// Une longueur de segment par articulation (en mètres).
/// Chaque angle articulaire est exprimé *relativement* au segment précédent.
struct PlanarArm {
    std::vector<double> link_lengths;

    [[nodiscard]] std::size_t dof() const { return link_lengths.size(); }
};

/// Transformation homogène 2D (matrice 3x3) d'un segment :
/// rotation d'angle `theta` (rad), PUIS translation de `length` le long de l'axe x local.
///
/// C'est l'analogue 2D de la matrice 4x4 que tu connais :
///   [ cos  -sin   tx ]
///   [ sin   cos   ty ]
///   [  0     0     1 ]
/// bloc rotation 2x2 en haut à gauche, colonne de translation, dernière ligne (0 0 1).
///
/// À IMPLÉMENTER (Phase 0).
[[nodiscard]] Eigen::Matrix3d link_transform(double theta, double length);

/// Cinématique directe (FK) : à partir des angles articulaires (rad), calcule la pose
/// de l'effecteur dans le repère de la base.
///
/// Précondition : angles.size() == arm.dof().
///
/// À IMPLÉMENTER (Phase 0) : chaîne les link_transform() puis lit la pose finale.
[[nodiscard]] Pose2D forward_kinematics(const PlanarArm& arm,
                                        const std::vector<double>& angles);

}  // namespace kin
