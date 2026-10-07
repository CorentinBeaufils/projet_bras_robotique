// Phase 1.5 - Marche C : IHM de supervision complète.
// À gauche le flux annoté, à droite : pose, télémétrie, Pause, Arrêt d'urgence, Réarmer.
//
// Option de test : --slow-gui ralentit volontairement l'affichage (50 ms par image)
// pour simuler une GUI plus lente que la caméra et observer la latence.

#include <chrono>
#include <cmath>
#include <cstring>

#include <QApplication>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

#include "camera_worker.hpp"
#include "perception_types.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    bool slow_gui = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--slow-gui") == 0) {
            slow_gui = true;
        }
    }

    // Autorise nos types maison à franchir une connexion queued (worker -> GUI).
    qRegisterMetaType<MarkerPose>();
    qRegisterMetaType<Telemetry>();

    // ------------------------------- Mise en page -------------------------------
    auto* window = new QWidget;
    window->setWindowTitle("Phase 1.5 - Supervision");
    auto* layout = new QHBoxLayout(window);

    auto* video = new QLabel("En attente du flux camera...");
    video->setFixedSize(640, 480);
    video->setAlignment(Qt::AlignCenter);

    auto* side = new QVBoxLayout;
    auto* pose_label = new QLabel("Aucun marqueur");
    auto* telemetry_label = new QLabel("Telemetrie : -");
    auto* pause_button = new QPushButton("Pause");
    pause_button->setCheckable(true);
    auto* estop_button = new QPushButton("ARRET D'URGENCE");
    estop_button->setMinimumHeight(60);
    estop_button->setStyleSheet("background-color: #c62828; color: white; font-weight: bold;");
    auto* reset_button = new QPushButton("Rearmer");
    reset_button->setEnabled(false);

    side->addWidget(pose_label);
    side->addWidget(telemetry_label);
    side->addStretch();
    side->addWidget(pause_button);
    side->addWidget(estop_button);
    side->addWidget(reset_button);

    layout->addWidget(video);
    layout->addLayout(side);
    window->setMinimumWidth(900);
    window->show();

    // ------------------------------ Thread worker -------------------------------
    auto* thread = new QThread;
    auto* worker = new CameraWorker(0);
    worker->moveToThread(thread);
    QObject::connect(thread, &QThread::started, worker, &CameraWorker::process);

    // ------------------- Worker -> GUI : signaux (queued) ------------------------
    double last_latency_ms = 0.0;  // n'est touché QUE sur le thread GUI

    QObject::connect(worker, &CameraWorker::frameReady, video,
                     [video, worker, slow_gui, &last_latency_ms](const QImage& img, qint64 capture_ns) {
        if (slow_gui) {
            QThread::msleep(50);  // simulation d'une GUI lente (test uniquement)
        }
        video->setPixmap(QPixmap::fromImage(img));
        const qint64 now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch()).count();
        last_latency_ms = static_cast<double>(now_ns - capture_ns) / 1e6;
        worker->frameDisplayed();  // atomique : signale au worker qu'il peut envoyer la suivante
    });

    QObject::connect(worker, &CameraWorker::poseReady, pose_label, [pose_label](const MarkerPose& p) {
        if (!p.visible) {
            pose_label->setText("Aucun marqueur");
            return;
        }
        const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        pose_label->setText(QString("id %1\n\nx = %2 m\ny = %3 m\nz = %4 m\n\ndistance = %5 m")
                                .arg(p.id)
                                .arg(p.x, 0, 'f', 3)
                                .arg(p.y, 0, 'f', 3)
                                .arg(p.z, 0, 'f', 3)
                                .arg(d, 0, 'f', 3));
    });

    QObject::connect(worker, &CameraWorker::telemetryReady, telemetry_label,
                     [telemetry_label, &last_latency_ms](const Telemetry& t) {
        QString state = "EN MARCHE";
        if (t.emergency_stop) {
            state = "ARRET D'URGENCE";
        } else if (t.paused) {
            state = "PAUSE";
        }
        telemetry_label->setText(QString("Etat : %1\n\nFPS : %2\nTraitement : %3 ms\n"
                                         "Latence capture->affichage : %4 ms\nImages jetees : %5")
                                     .arg(state)
                                     .arg(t.fps, 0, 'f', 1)
                                     .arg(t.processing_ms, 0, 'f', 1)
                                     .arg(last_latency_ms, 0, 'f', 1)
                                     .arg(t.frames_dropped));
    });

    // --------- GUI -> worker : appels DIRECTS qui n'écrivent que des atomiques ---------
    // Le contexte de ces lambdas est un bouton (thread GUI) : elles s'exécutent donc sur le
    // thread GUI, immédiatement, sans passer par la file d'événements du worker.
    QObject::connect(pause_button, &QPushButton::toggled, pause_button, [worker, pause_button](bool on) {
        worker->setPaused(on);
        pause_button->setText(on ? "Reprendre" : "Pause");
    });

    QObject::connect(estop_button, &QPushButton::clicked, estop_button, [worker, reset_button]() {
        worker->triggerEmergencyStop();  // effet immédiat, verrouillé
        reset_button->setEnabled(true);
    });

    QObject::connect(reset_button, &QPushButton::clicked, reset_button, [worker, reset_button]() {
        worker->resetEmergencyStop();    // réarmement volontaire, seul moyen de lever l'arrêt
        reset_button->setEnabled(false);
    });

    // ------------------------------ Cycle de vie --------------------------------
    QObject::connect(worker, &CameraWorker::finished, thread, &QThread::quit);
    QObject::connect(thread, &QThread::finished, worker, &QObject::deleteLater);
    QObject::connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    thread->start();
    const int rc = app.exec();

    worker->stop();
    thread->quit();
    thread->wait();
    return rc;
}
