// Phase 1 - Marche C : calibration de la caméra (intrinsèques).
//
// TU écris les DEUX blocs marqués TODO. Le reste (boucle, capture des vues,
// sauvegarde du résultat) est fourni pour que le programme tourne.
//
// Principe : on montre un échiquier de géométrie CONNUE à la caméra sous
// beaucoup d'angles/distances. Pour chaque vue on associe les coins 3D connus
// (board_object_points) aux coins 2D détectés dans l'image. cv::calibrateCamera
// remonte alors la matrice K et les coefficients de distorsion.
//
// Commandes : ESPACE = capturer la vue courante, c = calibrer, q/Echap = quitter.

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

// Coins INTERNES de l'échiquier (pas les cases).
constexpr int kCols = 9;
constexpr int kRows = 6;

// MESURE le côté d'une case sur ta mire IMPRIMÉE (en mètres) et ajuste.
// La valeur ci-dessous n'est correcte que si tes cases font pile 25 mm.
constexpr float kSquareMeters = 0.0227f;

// Positions 3D des coins de l'échiquier dans SON repère (z = 0), dans l'ordre
// où findChessboardCorners les renvoie (x d'abord, puis y).
std::vector<cv::Point3f> board_object_points() {
    std::vector<cv::Point3f> pts;
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            pts.emplace_back(x * kSquareMeters, y * kSquareMeters, 0.0f);
        }
    }
    return pts;
}

}  // namespace

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera 0.\n";
        return 1;
    }
    const cv::Size pattern(kCols, kRows);
    std::vector<std::vector<cv::Point2f>> image_points;   // coins 2D par vue
    std::vector<std::vector<cv::Point3f>> object_points;  // coins 3D par vue
    cv::Size image_size;

    std::cout << "ESPACE = capturer une vue | c = calibrer | q = quitter\n";

    cv::Mat frame, gray;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        image_size = frame.size();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = false;

        // ================= TODO (marche C-1) : détection des coins =================
        // 1. cv::findChessboardCorners(gray, pattern, corners, ...) -> renvoie un bool.
        // 2. Si trouvé, affine la précision au sous-pixel avec cv::cornerSubPix(gray, corners, ...).
        // 3. Superpose le résultat avec cv::drawChessboardCorners(frame, pattern, corners, found).
        // Mets `found` à true quand des coins sont détectés.
        // ==========================================================================
        found = cv::findChessboardCorners(gray, pattern, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::drawChessboardCorners(frame, pattern, corners, found);
        }
        cv::putText(frame, "vues capturees : " + std::to_string(object_points.size()),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    cv::Scalar(0, 255, 0), 2);
        cv::imshow("Phase 1 - Marche C : calibration", frame);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == ' ' && found) {
            object_points.push_back(board_object_points());
            image_points.push_back(corners);
            std::cout << "Vue capturee (" << object_points.size() << ").\n";
        }
        if (key == 'c') {
            if (object_points.size() < 8) {
                std::cout << "Pas assez de vues (vise >= 8, variees en angle/distance).\n";
                continue;
            }
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            std::vector<cv::Mat> rvecs;
            std::vector<cv::Mat> tvecs;
            double reproj_error = -1.0;

            // ================= TODO (marche C-2) : calibration =================
            // reproj_error = cv::calibrateCamera(object_points, image_points, image_size,
            //                                    camera_matrix, dist_coeffs, rvecs, tvecs);
            // ==================================================================
            reproj_error = cv::calibrateCamera(object_points, image_points, image_size,
                                               camera_matrix, dist_coeffs, rvecs, tvecs);

            std::cout << "Erreur de reprojection = " << reproj_error << " px\n";
            std::cout << "K =\n" << camera_matrix << "\n";

            cv::FileStorage fs("camera_intrinsics.yml", cv::FileStorage::WRITE);
            fs << "image_width" << image_size.width;
            fs << "image_height" << image_size.height;
            fs << "camera_matrix" << camera_matrix;
            fs << "distortion_coefficients" << dist_coeffs;
            fs.release();
            std::cout << "Intrinseques sauvees dans camera_intrinsics.yml\n";
        }
    }
    return 0;
}
