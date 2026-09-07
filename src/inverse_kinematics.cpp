#include "kinematics/inverse_kinematics.hpp"

#include <cmath>     // std::sqrt, std::acos, std::atan2, std::sin, std::cos
#include <algorithm> // std::clamp

namespace kin {

std::optional<IkSolution> inverse_kinematics_2link(double l1, double l2, double x,
                                                   double y, Elbow elbow) {
    // TODO(phase0) : IK analytique 2 segments.
    //   1. d = distance base->cible. Si hors de la couronne |l1-l2| <= d <= l1+l2,
    //      renvoie std::nullopt.
    //   2. theta2 par la loi des cosinus (std::acos). Le SIGNE dépend de `elbow`
    //      (Up -> +, Down -> -).
    //   3. theta1 = atan2(y, x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2)).
    //
    //   PIÈGE À GÉRER : au bord exact de l'espace atteignable, les erreurs
    //   flottantes peuvent pousser l'argument de acos très légèrement hors de
    //   [-1, 1] -> acos renvoie NaN. Comment tu t'en protèges proprement ?

    const double diff_L = l1-l2;
    const double min_dist = diff_L * diff_L;

    const double sum_L = l1 + l2;
    const double max_dist = sum_L * sum_L;

    const double target_dist_sq = x * x + y * y;

    if (!(target_dist_sq >= min_dist && target_dist_sq <= max_dist)) {
        return std::nullopt;
    }

    double theta_2 = std::acos(std::clamp((target_dist_sq - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1.0, 1.0));
    
    if (elbow == Elbow::Down) {
        theta_2 = -theta_2;
    }

    const double k1 = l1 + l2 * std::cos(theta_2);
    const double k2 = l2 * std::sin(theta_2);
    const double theta_1 = std::atan2(y, x) - std::atan2(k2, k1);

    return IkSolution{theta_1, theta_2};

}

}  // namespace kin
