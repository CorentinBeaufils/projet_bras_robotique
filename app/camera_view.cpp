// Phase 1 - Marche A : ouvrir la webcam et afficher le flux.
// Aucune détection encore : on valide juste qu'OpenCV voit la caméra.
//
// Usage : camera_view [index_camera]   (0 = webcam intégrée par défaut)

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char** argv) {
    const int cam_index = (argc > 1) ? std::stoi(argv[1]) : 0;

    cv::VideoCapture cap(cam_index);
    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la camera " << cam_index
                  << " (essaie un autre index : camera_view 1).\n";
        return 1;
    }

    std::cout << "Flux ouvert (" << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "). 'q' ou Echap pour quitter.\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Image vide, arret.\n";
            break;
        }
        cv::imshow("Phase 1 - Marche A : flux webcam", frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 27 = Echap
            break;
        }
    }
    return 0;
}
