// Phase 1 - Marche D : position 3D d'un marqueur ArUco (LE JALON).
//
// Charge les intrinsèques (camera_intrinsics.yml produit par calibrate), détecte
// le marqueur, et estime sa POSE 3D dans le repère caméra. Affiche les axes du
// marqueur et sa position (x, y, z) en mètres.
//
// TU écris le bloc TODO (l'estimation de pose). Le reste est fourni.
//
// Prérequis : lance d'abord ./build/calibrate pour générer camera_intrinsics.yml.

#include <iostream>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

//  MESURE le côté du carré NOIR de ton marqueur imprimé/affiché (en mètres).
// C'est cette taille réelle qui donne l'échelle métrique (lève l'ambiguïté petit-loin / grand-près).
constexpr float kMarkerMeters = 0.1545f;

}  // namespace

int main() {
    // 1) Charger les intrinsèques.
    cv::FileStorage fs("camera_intrinsics.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "camera_intrinsics.yml introuvable : lance d'abord ./build/calibrate.\n";
        return 1;
    }
    cv::Mat camera_matrix, dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera 0.\n";
        return 1;
    }
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    // Coins 3D du marqueur dans son repere local (origine au centre).
    const float half = kMarkerMeters * 0.5f;
    const std::vector<cv::Point3f> marker_object_points = {
        {-half, half, 0.0f},
        {half, half, 0.0f},
        {half, -half, 0.0f},
        {-half, -half, 0.0f},
    };

    std::cout << "Pose ArUco. Marqueur = " << kMarkerMeters << " m. 'q'/Echap pour quitter.\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(frame, corners, ids);
        //change le code depuis le debut du if pour utiliser la version 4.7+ de l'api OpenCV 
        
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (std::size_t i = 0; i < corners.size(); ++i) {
                cv::Vec3d rvec, tvec;
                const bool ok = cv::solvePnP(marker_object_points, corners[i], camera_matrix,
                                             dist_coeffs, rvec, tvec, false,
                                             cv::SOLVEPNP_IPPE_SQUARE);
                if (!ok) {
                    continue;
                }

                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec,
                                  kMarkerMeters * 0.5f);

                const double dist = cv::norm(tvec);
                char buf[128];
                std::snprintf(buf, sizeof(buf), "id %d: x=%.3f y=%.3f z=%.3f (d=%.3f m)",
                              ids[i], tvec[0], tvec[1], tvec[2], dist);
                cv::putText(frame, buf, cv::Point(10, 30 + 25 * static_cast<int>(i)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
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
