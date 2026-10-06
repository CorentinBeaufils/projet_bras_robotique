#pragma once

#include <atomic>

#include <QImage>
#include <QObject>

#include "perception_types.hpp"

// Capture + perception dans un thread SÉPARÉ de la GUI.
// N'accède à aucun widget : émet des signaux que la fenêtre reçoit (queued).
class CameraWorker : public QObject {
    Q_OBJECT

public:
    explicit CameraWorker(int cam_index = 0, QObject* parent = nullptr);

public slots:
    void process();  // boucle capture + détection (lancée au démarrage du thread)
    void stop();

signals:
    void frameReady(const QImage& image);   // image annotée -> GUI
    void poseReady(const MarkerPose& pose);  // pose 3D -> GUI (télémétrie)
    void finished();

private:
    int cam_index_;
    std::atomic<bool> running_{true};
};
