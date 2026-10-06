#include "camera_worker.hpp"

#include <vector>
#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "frame_convert.hpp"

namespace {
// TA vraie taille de marqueur (côté extérieur du carré noir, en mètres) — cf. marche D.
constexpr float kMarkerMeters = 0.1545f;
}  // namespace

CameraWorker::CameraWorker(int cam_index, QObject* parent)
    : QObject(parent), cam_index_(cam_index) {}

void CameraWorker::stop() { running_ = false; }

void CameraWorker::process() {
    // Intrinsèques produites par calibrate (camera_intrinsics.yml, dossier courant).
    cv::Mat camera_matrix, dist_coeffs;
    {
        cv::FileStorage fs("camera_intrinsics.yml", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "camera_intrinsics.yml introuvable : lance d'abord ./build/calibrate.\n";
            emit finished();
            return;
        }
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        fs.release();
    }

    if(camera_matrix.empty() || dist_coeffs.empty()) {
        std::cerr << "Erreur : intrinseques de la camera non valides.\n";
        emit finished();
        return;
    }

    cv::VideoCapture cap(cam_index_);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera " << cam_index_ << ".\n";
        emit finished();
        return;
    }
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    const float h = kMarkerMeters * 0.5f;
    const std::vector<cv::Point3f> objp = {{-h, h, 0},
                                            {h, h, 0},
                                            {h, -h, 0},
                                            {-h, -h, 0}};

    cv::Mat frame;
    while (running_) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        MarkerPose pose;  // visible = false par défaut
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        
        detector.detectMarkers(frame, corners, ids);
        const float h = kMarkerMeters * 0.5f;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            cv::Vec3d rvec, tvec;
            bool ok = cv::solvePnP(objp, corners[i], camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            if (ok) {
                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, h);
            }
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            if (i == 0) { pose.visible = true; pose.id = ids[0]; pose.x = tvec[0]; pose.y = tvec[1]; pose.z = tvec[2]; }
        }
        
        emit poseReady(pose);
        emit frameReady(toQImage(frame));
    }
    emit finished();
}
