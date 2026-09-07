#pragma once

#include <optional>

namespace kin {

/// Configuration du coude pour un bras plan à 2 segments.
/// Up   : angle du coude positif (theta2 >= 0).
/// Down : angle du coude négatif (theta2 <= 0).
enum class Elbow { Up, Down };

/// Solution d'IK : les deux angles articulaires (radians), relatifs au segment précédent.
struct IkSolution {
    double theta1 = 0.0;
    double theta2 = 0.0;
};

/// Cinématique inverse analytique d'un bras plan à 2 segments.
///
/// Entrées : longueurs l1, l2 ; cible (x, y) dans le repère de la base ; choix du coude.
/// Retour  : les deux angles, ou std::nullopt si la cible est HORS de portée,
///           c'est-à-dire hors de la couronne  |l1 - l2| <= d <= l1 + l2
///           où d = distance base->cible.
///
/// À IMPLÉMENTER (Phase 0).
[[nodiscard]] std::optional<IkSolution> inverse_kinematics_2link(
    double l1, double l2, double x, double y, Elbow elbow);

}  // namespace kin
