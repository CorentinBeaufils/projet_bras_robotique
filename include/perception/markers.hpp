// include/perception/markers.hpp
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <optional>
#include <string>
#include <vector>

namespace perception {

struct CameraIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
};

struct Pose {
    cv::Vec3d rvec;  // rotation (Rodrigues)
    cv::Vec3d tvec;  // position du marqueur dans le repère caméra, en mètres
};

struct MarkerObservation {
    int id = -1;
    std::vector<cv::Point2f> corners;  // 4 coins dans l'image
    std::optional<Pose> pose;          // vide si solvePnP a échoué
};

// Lit camera_intrinsics.yml. nullopt si le fichier ou une matrice manque.
std::optional<CameraIntrinsics> load_intrinsics(const std::string& path);

// Coins 3D d'un marqueur carré de côté `side` (mètres), centré, dans l'ordre d'ArUco.
std::vector<cv::Point3f> marker_object_points(float side);

// Pose d'UN marqueur à partir de ses 4 coins image. Pur calcul, aucune image.
std::optional<Pose> estimate_marker_pose(const std::vector<cv::Point2f>& corners,
                                         const CameraIntrinsics& intrinsics, float side);

// Détecte tous les marqueurs d'une image et estime la pose de chacun.
class MarkerDetector {
public:
    MarkerDetector(cv::aruco::PredefinedDictionaryType dictionary, CameraIntrinsics intrinsics,
                   float marker_side);
    std::vector<MarkerObservation> detect(const cv::Mat& image) const;

private:
    cv::aruco::ArucoDetector detector_;
    CameraIntrinsics intrinsics_;
    float marker_side_;
};

}  // namespace perception