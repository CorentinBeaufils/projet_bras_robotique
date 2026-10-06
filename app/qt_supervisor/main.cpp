// Phase 1.5 - Marche B : IHM de supervision.
// À gauche le flux annoté (marqueurs + axes 3D), à droite la télémétrie de pose.
// La perception tourne dans un thread séparé ; frames et poses arrivent par signaux.

#include <cmath>

#include <QApplication>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QString>
#include <QThread>
#include <QWidget>

#include "camera_worker.hpp"
#include "perception_types.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    // Autorise MarkerPose à franchir une connexion queued (worker -> GUI).
    qRegisterMetaType<MarkerPose>();

    auto* window = new QWidget;
    window->setWindowTitle("Phase 1.5 - Supervision perception");
    auto* layout = new QHBoxLayout(window);

    auto* video = new QLabel("En attente du flux camera...");
    video->setFixedSize(640, 480);
    video->setAlignment(Qt::AlignCenter);

    auto* panel = new QLabel("Aucun marqueur");
    panel->setMinimumWidth(220);
    panel->setAlignment(Qt::AlignTop | Qt::AlignLeft);

    layout->addWidget(video);
    layout->addWidget(panel);
    window->show();

    auto* thread = new QThread;
    auto* worker = new CameraWorker(0);
    worker->moveToThread(thread);

    QObject::connect(thread, &QThread::started, worker, &CameraWorker::process);

    QObject::connect(worker, &CameraWorker::frameReady, video,
                     [video](const QImage& img) { video->setPixmap(QPixmap::fromImage(img)); });

    QObject::connect(worker, &CameraWorker::poseReady, panel, [panel](const MarkerPose& p) {
        if (!p.visible) {
            panel->setText("Aucun marqueur");
            return;
        }
        const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        panel->setText(QString("id %1\n\nx = %2 m\ny = %3 m\nz = %4 m\n\ndistance = %5 m")
                           .arg(p.id)
                           .arg(p.x, 0, 'f', 3)
                           .arg(p.y, 0, 'f', 3)
                           .arg(p.z, 0, 'f', 3)
                           .arg(d, 0, 'f', 3));
    });

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
