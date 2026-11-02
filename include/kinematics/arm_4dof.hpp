#pragma once

#include <variant>
#include <vector>

#include <Eigen/Core>

namespace kin {

// Bras 4 axes : base (lacet) + épaule, coude, poignet (tangage, dans le plan vertical du bras).
// Conventions (radians, repère de la base : z vers le haut, x devant le bras à base = 0) :
//   base     : rotation autour de z, 0 = vers +x, positif vers +y ;
//   shoulder : angle du bras par rapport à l'horizontale, positif vers le haut ;
//   elbow    : angle de l'avant-bras par rapport au bras (0 = aligné),
//              positif dans le même sens que l'épaule (vers le haut dans le plan vertical) ;
//   wrist    : angle de la main par rapport à l'avant-bras (0 = alignée),
//              positif dans le même sens que l'épaule (vers le haut dans le plan vertical).
struct Arm4DofGeometry {     // mètres
    double shoulder_height;  // base -> axe de l'épaule
    double upper_arm;        // l1 : épaule -> coude
    double forearm;          // l2 : coude -> poignet
    double hand;             // l3 : poignet -> pointe de l'outil
};

struct Arm4DofJoints {
    double base;
    double shoulder;
    double elbow;
    double wrist;
};

// Position de la pointe de l'outil dans le repère de la base.
[[nodiscard]] Eigen::Vector3d forward_kinematics_4dof(const Arm4DofGeometry& arm,
                                                      const Arm4DofJoints& q);

// --- Résultat de l'IK : exactement UN des trois cas, jamais un mélange. ---
struct Unreachable {};
struct IkSolutions {
    std::vector<Arm4DofJoints> joints;
};
struct IkBaseSingular {  // cible sur l'axe z : base libre, fixée à current_base
    std::vector<Arm4DofJoints> joints;
};
using IkResult = std::variant<Unreachable, IkSolutions, IkBaseSingular>;

// approach_pitch : angle ABSOLU de la main par rapport à l'horizontale (-pi/2 = vers le bas).
// current_base   : angle de base actuel, utilisé seulement si la base est indéterminée.
// Renvoie TOUTES les solutions mathématiques, sans tenir compte des butées articulaires.
IkResult inverse_kinematics_4dof(const Arm4DofGeometry& arm, const Eigen::Vector3d& target,
                                 double approach_pitch, double current_base);

}  // namespace kin
