// Démo Phase 0 : un bras plan 2 segments qui va se poser sur une cible.
//
// Utilise la lib `kinematics` :
//   - inverse_kinematics_2link() pour trouver les angles qui atteignent la cible,
//   - link_transform() (chaîné) pour placer chaque articulation à chaque image.
//
// Sortie : un fichier SVG *animé* (aucune dépendance graphique) montrant le bras
// interpoler depuis sa pose de repos jusqu'à la cible. Ouvre-le dans un navigateur.
//
// Usage : arm_demo [x_cible] [y_cible] [fichier_sortie.svg]

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "kinematics/inverse_kinematics.hpp"
#include "kinematics/planar_arm.hpp"

namespace {

// Longueurs du bras (mètres). Bras 2 segments égaux pour la démo.
constexpr double kL1 = 1.0;
constexpr double kL2 = 1.0;

// Images de l'animation (interpolation repos -> cible).
constexpr int kFrames = 32;

// Rendu.
constexpr double kImg = 480.0;  // image carrée, en pixels

// Positions de TOUTES les articulations (base, coude, effecteur) dans le repère base,
// en chaînant link_transform() — exactement la même mécanique que la FK.
std::vector<Eigen::Vector2d> joint_positions(const kin::PlanarArm& arm,
                                             const std::vector<double>& angles) {
    std::vector<Eigen::Vector2d> pts;
    pts.emplace_back(0.0, 0.0);  // base
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    for (std::size_t i = 0; i < angles.size(); ++i) {
        T = T * kin::link_transform(angles[i], arm.link_lengths[i]);
        pts.emplace_back(T(0, 2), T(1, 2));
    }
    return pts;
}

}  // namespace

int main(int argc, char** argv) {
    const double target_x = (argc > 1) ? std::stod(argv[1]) : 1.2;
    const double target_y = (argc > 2) ? std::stod(argv[2]) : 0.8;
    const std::string out_path = (argc > 3) ? argv[3] : "arm_demo.svg";

    // 1) IK : les angles qui posent l'effecteur sur la cible (coude haut).
    const auto sol = kin::inverse_kinematics_2link(kL1, kL2, target_x, target_y, kin::Elbow::Up);
    if (!sol.has_value()) {
        std::cerr << "Cible (" << target_x << ", " << target_y << ") hors de portee.\n";
        return 1;
    }

    const kin::PlanarArm arm{{kL1, kL2}};

    // 2) Interpolation des angles : pose de repos {0,0} -> solution IK.
    const std::vector<double> rest{0.0, 0.0};
    const std::vector<double> goal{sol->theta1, sol->theta2};

    // Repère écran : la base est à gauche-milieu, y vers le haut.
    const double reach = kL1 + kL2;
    const double scale = (kImg * 0.42) / reach;
    const double base_px = kImg * 0.15;
    const double base_py = kImg * 0.5;
    auto to_px_x = [&](double x) {
        return base_px + x * scale;
    };
    auto to_px_y = [&](double y) {
        return base_py - y * scale;
    };  // flip vertical

    // Construit, pour chaque articulation animée, la liste de valeurs SMIL "v0;v1;...".
    std::ostringstream j1x, j1y, j2x, j2y;  // coude (j1) et effecteur (j2)
    for (int f = 0; f < kFrames; ++f) {
        const double t = static_cast<double>(f) / (kFrames - 1);
        std::vector<double> ang(2);
        ang[0] = rest[0] + t * (goal[0] - rest[0]);
        ang[1] = rest[1] + t * (goal[1] - rest[1]);
        const auto p = joint_positions(arm, ang);
        const char* sep = (f == 0) ? "" : ";";
        j1x << sep << to_px_x(p[1].x());
        j1y << sep << to_px_y(p[1].y());
        j2x << sep << to_px_x(p[2].x());
        j2y << sep << to_px_y(p[2].y());
    }

    // 3) Écrit le SVG animé.
    std::ofstream svg(out_path);
    if (!svg) {
        std::cerr << "Impossible d'ecrire " << out_path << "\n";
        return 1;
    }
    const std::string dur = "2.4s";
    const std::string anim =
        "\" dur=\"" + dur + "\" repeatCount=\"indefinite\" calcMode=\"linear\"/>";

    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << kImg << "\" height=\"" << kImg
        << "\" viewBox=\"0 0 " << kImg << " " << kImg << "\">\n";
    svg << "  <rect width=\"100%\" height=\"100%\" fill=\"#0f1720\"/>\n";
    // Portee max (cercle repere).
    svg << "  <circle cx=\"" << base_px << "\" cy=\"" << base_py << "\" r=\"" << reach * scale
        << "\" fill=\"none\" stroke=\"#243244\" stroke-width=\"1\" stroke-dasharray=\"4 4\"/>\n";
    // Cible.
    svg << "  <circle cx=\"" << to_px_x(target_x) << "\" cy=\"" << to_px_y(target_y)
        << "\" r=\"7\" fill=\"none\" stroke=\"#ff5d5d\" stroke-width=\"2\"/>\n";
    svg << "  <line x1=\"" << to_px_x(target_x) - 10 << "\" y1=\"" << to_px_y(target_y)
        << "\" x2=\"" << to_px_x(target_x) + 10 << "\" y2=\"" << to_px_y(target_y)
        << "\" stroke=\"#ff5d5d\" stroke-width=\"1.5\"/>\n";
    svg << "  <line x1=\"" << to_px_x(target_x) << "\" y1=\"" << to_px_y(target_y) - 10
        << "\" x2=\"" << to_px_x(target_x) << "\" y2=\"" << to_px_y(target_y) + 10
        << "\" stroke=\"#ff5d5d\" stroke-width=\"1.5\"/>\n";
    // Segment 1 : base (fixe) -> coude (animé).
    svg << "  <line x1=\"" << base_px << "\" y1=\"" << base_py << "\" x2=\"" << to_px_x(0)
        << "\" y2=\"" << to_px_y(0)
        << "\" stroke=\"#4da3ff\" stroke-width=\"6\" stroke-linecap=\"round\">\n";
    svg << "    <animate attributeName=\"x2\" values=\"" << j1x.str() << anim << "\n";
    svg << "    <animate attributeName=\"y2\" values=\"" << j1y.str() << anim << "\n";
    svg << "  </line>\n";
    // Segment 2 : coude (animé) -> effecteur (animé).
    svg << "  <line x1=\"" << to_px_x(0) << "\" y1=\"" << to_px_y(0) << "\" x2=\"" << to_px_x(0)
        << "\" y2=\"" << to_px_y(0)
        << "\" stroke=\"#7ee081\" stroke-width=\"6\" stroke-linecap=\"round\">\n";
    svg << "    <animate attributeName=\"x1\" values=\"" << j1x.str() << anim << "\n";
    svg << "    <animate attributeName=\"y1\" values=\"" << j1y.str() << anim << "\n";
    svg << "    <animate attributeName=\"x2\" values=\"" << j2x.str() << anim << "\n";
    svg << "    <animate attributeName=\"y2\" values=\"" << j2y.str() << anim << "\n";
    svg << "  </line>\n";
    // Articulations.
    svg << "  <circle cx=\"" << base_px << "\" cy=\"" << base_py
        << "\" r=\"6\" fill=\"#e8eef5\"/>\n";
    svg << "  <circle cx=\"" << to_px_x(0) << "\" cy=\"" << to_px_y(0)
        << "\" r=\"5\" fill=\"#e8eef5\">\n";
    svg << "    <animate attributeName=\"cx\" values=\"" << j1x.str() << anim << "\n";
    svg << "    <animate attributeName=\"cy\" values=\"" << j1y.str() << anim << "\n";
    svg << "  </circle>\n";
    svg << "  <circle cx=\"" << to_px_x(0) << "\" cy=\"" << to_px_y(0)
        << "\" r=\"5\" fill=\"#ffd166\">\n";
    svg << "    <animate attributeName=\"cx\" values=\"" << j2x.str() << anim << "\n";
    svg << "    <animate attributeName=\"cy\" values=\"" << j2y.str() << anim << "\n";
    svg << "  </circle>\n";
    svg << "</svg>\n";

    // Contrôle : la FK des angles solution doit retomber sur la cible.
    const kin::Pose2D end = kin::forward_kinematics(arm, goal);
    std::cerr << "Cible (" << target_x << ", " << target_y << ") | effecteur atteint (" << end.x
              << ", " << end.y << ") | angles = (" << sol->theta1 << ", " << sol->theta2 << ")\n";
    std::cerr << "SVG anime ecrit dans " << out_path << "\n";
    return 0;
}
