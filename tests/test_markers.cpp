// Tests de la lib perception : logique pure, sans caméra ni image.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "perception/markers.hpp"

using Catch::Approx;

namespace {

// Une caméra de test, du même ordre que ta webcam (640x480), sans distorsion.
perception::CameraIntrinsics test_intrinsics() {
    perception::CameraIntrinsics in;
    in.camera_matrix = (cv::Mat_<double>(3, 3) << 642.6, 0, 311.2,  //
                        0, 642.7, 242.3,                            //
                        0, 0, 1);
    in.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    return in;
}

}  // namespace

TEST_CASE("marker_object_points : 4 coins centrés, côté respecté", "[perception]") {
    // À écrire : appelle marker_object_points(0.1f) et vérifie
    //   - qu'il renvoie 4 points ;
    //   - que deux coins voisins sont à 0.1 m l'un de l'autre (cv::norm de la différence) ;
    //   - que le barycentre vaut (0, 0, 0) et que z vaut 0 pour chaque point.
    std::vector<cv::Point3f> points = perception::marker_object_points(0.1f);
    REQUIRE(points.size() == 4);
    // Vérifie que deux coins voisins sont à 0.1 m l'un de l'autre
    REQUIRE(cv::norm(points[0] - points[1]) == Approx(0.1f));
    REQUIRE(cv::norm(points[1] - points[2]) == Approx(0.1f));
    cv::Point3f barycentre(0, 0, 0);
    for (const auto& p : points) {
        barycentre += p;
        REQUIRE(p.z == Approx(0.0f));
    }
    barycentre *= 0.25f;
    REQUIRE(barycentre.x == Approx(0.0f));
    REQUIRE(barycentre.y == Approx(0.0f));
    REQUIRE(barycentre.z == Approx(0.0f));
}

TEST_CASE("estimate_marker_pose : aller-retour projection -> estimation", "[perception]") {
    // Le test qui compte, le même principe que l'aller-retour FK∘IK de la Phase 0.
    //   1. Choisis une pose connue : rvec_vrai et tvec_vrai (cv::Vec3d), par exemple un
    //      marqueur à 40 cm de la caméra, un peu décalé et un peu tourné.
    //   2. Calcule les coins que la caméra « verrait » :
    //        cv::projectPoints(objp, rvec_vrai, tvec_vrai, K, dist, coins_image);
    //      avec objp = marker_object_points(côté) et coins_image un std::vector<cv::Point2f>.
    //   3. Donne coins_image à estimate_marker_pose et vérifie qu'il renvoie une valeur,
    //      et qu'on retrouve tvec_vrai et rvec_vrai composante par composante
    //      (Approx(...).margin(1e-6)).
    cv::Vec3d rvec_vrai(0.1, 0.2, 0.3);
    cv::Vec3d tvec_vrai(0.0, 0.0, 0.4);
    float marker_side = 0.1f;
    std::vector<cv::Point3f> objp = perception::marker_object_points(marker_side);
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Point2f> coins_image;
    cv::projectPoints(objp, rvec_vrai, tvec_vrai, K, dist, coins_image);
    perception::CameraIntrinsics intrinsics{K, dist};
    auto pose = perception::estimate_marker_pose(coins_image, intrinsics, marker_side);
    REQUIRE(pose);
    for (int i = 0; i < 3; ++i) {
        REQUIRE(pose->rvec[i] == Approx(rvec_vrai[i]).margin(1e-6));
        REQUIRE(pose->tvec[i] == Approx(tvec_vrai[i]).margin(1e-6));
    }
}

TEST_CASE("estimate_marker_pose : moins de 4 coins -> nullopt", "[perception]") {
    // À écrire : avec seulement 3 coins, la fonction doit renvoyer std::nullopt,
    // pas planter ni renvoyer une pose fantaisiste.
    std::vector<cv::Point2f> corners = {{0.0f, 0.0f}, {0.1f, 0.0f}, {0.1f, 0.1f}};
    perception::CameraIntrinsics intrinsics{cv::Mat::eye(3, 3, CV_64F),
                                            cv::Mat::zeros(5, 1, CV_64F)};
    auto pose = perception::estimate_marker_pose(corners, intrinsics, 0.1f);
    REQUIRE(!pose);
}

TEST_CASE("load_intrinsics : relit un fichier écrit comme le fait calibrate", "[perception]") {
    std::filesystem::path temp_path =
        std::filesystem::temp_directory_path() / "test_intrinsics.yml";
    {
        cv::FileStorage fs(temp_path.string(), cv::FileStorage::WRITE);
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
    }
    auto intrinsics = perception::load_intrinsics(temp_path.string());
    REQUIRE(intrinsics);
    REQUIRE(cv::norm(intrinsics->camera_matrix - cv::Mat::eye(3, 3, CV_64F)) == Approx(0.0));
    REQUIRE(cv::norm(intrinsics->dist_coeffs - cv::Mat::zeros(5, 1, CV_64F)) == Approx(0.0));
    auto intrinsics_nonexistent = perception::load_intrinsics("nonexistent_file.yml");
    REQUIRE(!intrinsics_nonexistent);
}

TEST_CASE("load_intrinsics : cle absente le fichier n'a que distortion_coefficients",
          "[perception]") {
    std::filesystem::path temp_path =
        std::filesystem::temp_directory_path() / "test_intrinsics_missing_camera_matrix.yml";
    {
        cv::FileStorage fs(temp_path.string(), cv::FileStorage::WRITE);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        fs << "distortion_coefficients" << dist_coeffs;
    }
    auto intrinsics = perception::load_intrinsics(temp_path.string());
    REQUIRE(!intrinsics);
}

TEST_CASE("detect : détecte les marqueurs présents dans l'image", "[perception]") {
    const perception::CameraIntrinsics intrinsics = test_intrinsics();
    const double fx = intrinsics.camera_matrix.at<double>(0, 0);

    const int id = 0;
    const int cote_px = 200;
    const float marker_side = 0.1545f;

    cv::Mat marqueur;
    cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), id,
                                   cote_px, marqueur);

    cv::Mat fond(480, 640, CV_8UC1, cv::Scalar(255));
    const cv::Rect zone((311 - cote_px / 2), (242 - cote_px / 2), cote_px, cote_px);
    marqueur.copyTo(fond(zone));

    const perception::MarkerDetector detector(cv::aruco::DICT_4X4_50, intrinsics, marker_side);
    const auto observations = detector.detect(fond);

    REQUIRE(observations.size() == 1);
    REQUIRE(observations.front().id == id);
    REQUIRE(observations.front().pose);
    REQUIRE(observations.front().pose->tvec[0] == Approx(0.0).margin(0.005));
    REQUIRE(observations.front().pose->tvec[1] == Approx(0.0).margin(0.005));
    REQUIRE(observations.front().pose->tvec[2] == Approx(fx * marker_side / cote_px).epsilon(0.01));
}

TEST_CASE("detect : image sans marqueurs", "[perception]") {
    const perception::CameraIntrinsics intrinsics = test_intrinsics();

    const perception::MarkerDetector detector(cv::aruco::DICT_4X4_50, intrinsics, 0.1545f);
    cv::Mat fond(480, 640, CV_8UC1, cv::Scalar(255));
    const auto observations = detector.detect(fond);

    REQUIRE(observations.empty());
}
