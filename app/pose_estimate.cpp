// Phase 1 - Marche D : position 3D d'un marqueur ArUco (LE JALON).
//
// Charge les intrinsèques (camera_intrinsics.yml produit par calibrate), détecte
// le marqueur, et estime sa POSE 3D dans le repère caméra. Affiche les axes du
// marqueur et sa position (x, y, z) en mètres.

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio.hpp>

#include "perception/markers.hpp"

namespace {

//  MESURE le côté du carré NOIR de ton marqueur imprimé/affiché (en mètres).
// C'est cette taille réelle qui donne l'échelle métrique (lève l'ambiguïté petit-loin /
// grand-près).
constexpr float kMarkerMeters = 0.1545f;

}  // namespace

int main() {
    auto intrinsics = perception::load_intrinsics("camera_intrinsics.yml");
    if (!intrinsics) {
        std::cerr << "Chargement des intrinsics échoué depuis camera_intrinsics.yml\n";
        return 1;
    }
    std::cout << "intrinsics chargés avec succès.\n";

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera 0.\n";
        return 1;
    }

    const perception::MarkerDetector marker_detector(cv::aruco::DICT_4X4_50, *intrinsics,
                                                     kMarkerMeters);

    std::cout << "Pose ArUco. Marqueur = " << kMarkerMeters << " m. 'q'/Echap pour quitter.\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        const std::vector<perception::MarkerObservation> observations =
            marker_detector.detect(frame);
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const auto& obs = observations[i];
            const std::vector<std::vector<cv::Point2f>> marker_corners = {obs.corners};
            const std::vector<int> marker_ids = {obs.id};
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            if (obs.pose) {
                cv::drawFrameAxes(frame, intrinsics->camera_matrix, intrinsics->dist_coeffs,
                                  obs.pose->rvec, obs.pose->tvec, kMarkerMeters * 0.5);

                const double x = obs.pose->tvec[0];
                const double y = obs.pose->tvec[1];
                const double z = obs.pose->tvec[2];
                const double distance_m = cv::norm(obs.pose->tvec);
                const cv::Point text_position(10, 30 + 20 * static_cast<int>(i));

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3);
                oss << "x=" << x << " y=" << y << " z=" << z << " | d=" << distance_m << " m";

                cv::putText(frame, oss.str(), text_position, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
        cv::imshow("Phase 1 - Marche D : pose 3D ArUco", frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    return 0;
}
