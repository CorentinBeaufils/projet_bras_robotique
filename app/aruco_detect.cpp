// Phase 1 - Marche B : détecter des marqueurs ArUco.
// API ArUco "legacy" d'OpenCV 4.6 (fonctions libres). Pas de calibration ici :
// on détecte et on surligne, sans estimer la pose 3D (ça, c'est la marche D).
//
// Usage :
//   aruco_detect                      -> webcam 0
//   aruco_detect 1                    -> webcam d'index 1
//   aruco_detect --image entree.png [sortie.png]   -> détecte sur une image fixe

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

// Dessine les marqueurs détectés et renvoie le nombre trouvé.
int detect_and_draw(cv::Mat& image, const cv::aruco::ArucoDetector& detector) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    detector.detectMarkers(image, corners, ids, rejected);
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
    }
    return static_cast<int>(ids.size());
}

}  // namespace

int main(int argc, char** argv) {
    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    const cv::aruco::ArucoDetector detector(dict, params);

    // --- Mode image fixe (utile pour tester sans caméra) ---
    if (argc >= 3 && std::string(argv[1]) == "--image") {
        const std::string in_path = argv[2];
        const std::string out_path = (argc >= 4) ? argv[3] : "aruco_out.png";
        cv::Mat image = cv::imread(in_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Impossible de lire l'image " << in_path << "\n";
            return 1;
        }
        const int n = detect_and_draw(image, detector);
        cv::imwrite(out_path, image);
        std::cout << n << " marqueur(s) detecte(s), resultat ecrit dans " << out_path << "\n";
        return 0;
    }

    // --- Mode webcam ---
    const int cam_index = (argc > 1) ? std::stoi(argv[1]) : 0;
    cv::VideoCapture cap(cam_index);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera " << cam_index << "\n";
        return 1;
    }
    std::cout << "Detection ArUco (DICT_4X4_50). 'q' ou Echap pour quitter.\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Image vide, arret.\n";
            break;
        }
        const int n = detect_and_draw(frame, detector);
        cv::putText(frame, "Marqueurs : " + std::to_string(n), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Phase 1 - Marche B : detection ArUco", frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }
    return 0;
}
