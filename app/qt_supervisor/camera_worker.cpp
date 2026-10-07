#include "camera_worker.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "frame_convert.hpp"

namespace {
// TA vraie taille de marqueur (côté extérieur du carré noir, en mètres) — cf. marche D.
constexpr float kMarkerMeters = 0.1545f;

using Clock = std::chrono::steady_clock;  // monotone : jamais de saut (≠ system_clock)

qint64 to_ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

// Bandeau rouge dessiné sur l'image tant que l'arrêt d'urgence est verrouillé.
void draw_emergency_banner(cv::Mat& frame) {
    cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols, 40), cv::Scalar(0, 0, 255),
                  cv::FILLED);
    cv::putText(frame, "ARRET D'URGENCE - rearmement requis", cv::Point(10, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}
}  // namespace

CameraWorker::CameraWorker(int cam_index, QObject* parent)
    : QObject(parent), cam_index_(cam_index) {}

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

    if (camera_matrix.empty() || dist_coeffs.empty()) {
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
    const std::vector<cv::Point3f> objp = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};

    // --- Télémétrie : on accumule sur une fenêtre d'environ 1 s, puis on émet. ---
    auto window_start = Clock::now();
    int frames_in_window = 0;
    double processing_ms_sum = 0.0;
    auto maybe_emit_telemetry = [&]() {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - window_start).count();
        if (elapsed_s < 1.0) {
            return;
        }
        Telemetry t;
        t.fps = frames_in_window / elapsed_s;
        t.processing_ms = frames_in_window > 0 ? processing_ms_sum / frames_in_window : 0.0;
        t.frames_dropped = frames_dropped_;
        t.paused = paused_;
        t.emergency_stop = emergency_stop_;
        emit telemetryReady(t);
        window_start = now;
        frames_in_window = 0;
        processing_ms_sum = 0.0;
    };

    cv::Mat frame;
    while (running_) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        const auto t_capture = Clock::now();

        // Pause : on continue de LIRE la caméra (sinon son tampon interne garde des
        // images périmées qu'on verrait à la reprise), mais on ne traite ni n'émet rien.
        if (paused_) {
            maybe_emit_telemetry();
            continue;
        }

        // ---------------------- ton bloc perception (inchangé) ----------------------
        MarkerPose pose;  // visible = false par défaut
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        detector.detectMarkers(frame, corners, ids);
        for (std::size_t i = 0; i < ids.size(); ++i) {
            cv::Vec3d rvec, tvec;
            bool ok = cv::solvePnP(objp, corners[i], camera_matrix, dist_coeffs, rvec, tvec, false,
                                   cv::SOLVEPNP_IPPE_SQUARE);
            if (!ok) {
                continue;
            }
            cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, h);

            if (!pose.visible) {
                pose.visible = true;
                pose.id = ids[i];
                pose.x = tvec[0];
                pose.y = tvec[1];
                pose.z = tvec[2];
            }
        }
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        // -----------------------------------------------------------------------------

        // Arrêt d'urgence verrouillé : la supervision continue (on voit toujours la
        // scène), mais l'état est affiché sur l'image. Plus tard : aucune commande
        // ne partira vers le bras tant que ce drapeau est levé.
        if (emergency_stop_) {
            draw_emergency_banner(frame);
        }

        processing_ms_sum +=
            std::chrono::duration<double, std::milli>(Clock::now() - t_capture).count();
        ++frames_in_window;

        if (!frame_in_flight_.exchange(true)) {
            emit poseReady(pose);
            emit frameReady(toQImage(frame), to_ns(t_capture));
        } else {
            ++frames_dropped_;
        }

        maybe_emit_telemetry();
    }
    emit finished();
}
