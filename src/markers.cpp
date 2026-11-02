#include "perception/markers.hpp"

#include <opencv2/calib3d.hpp>
#include <utility>

namespace perception {

std::optional<CameraIntrinsics> load_intrinsics(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return std::nullopt;
    }
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        return std::nullopt;
    }
    return CameraIntrinsics{camera_matrix, dist_coeffs};
}

std::vector<cv::Point3f> marker_object_points(float side) {
    float half_side = side / 2.0f;
    return {{-half_side, half_side, 0.0f},
            {half_side, half_side, 0.0f},
            {half_side, -half_side, 0.0f},
            {-half_side, -half_side, 0.0f}};
}

std::optional<Pose> estimate_marker_pose(const std::vector<cv::Point2f>& corners,
                                         const CameraIntrinsics& intrinsics, float side) {
    if (corners.size() != 4) {
        return std::nullopt;
    }
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    const std::vector<cv::Point3f> object_points = marker_object_points(side);
    // cv::SOLVEPNP_IPPE_SQUARE est une méthode spécifique pour LES MARQUEURS CAREES qui peut donner
    // de meilleurs résultats.
    const bool success =
        cv::solvePnP(object_points, corners, intrinsics.camera_matrix, intrinsics.dist_coeffs, rvec,
                     tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    if (!success) {
        return std::nullopt;
    }
    return Pose{rvec, tvec};
}

MarkerDetector::MarkerDetector(cv::aruco::PredefinedDictionaryType dictionary,
                               CameraIntrinsics intrinsics, float marker_side)
    : detector_(cv::aruco::getPredefinedDictionary(dictionary)),
      intrinsics_(std::move(intrinsics)),
      marker_side_(marker_side) {}

std::vector<MarkerObservation> MarkerDetector::detect(const cv::Mat& image) const {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(image, corners, ids);

    std::vector<MarkerObservation> observations;
    for (size_t i = 0; i < ids.size(); ++i) {
        auto pose = estimate_marker_pose(corners[i], intrinsics_, marker_side_);
        observations.push_back(MarkerObservation{ids[i], std::move(corners[i]), pose});
    }
    return observations;
}

}  // namespace perception