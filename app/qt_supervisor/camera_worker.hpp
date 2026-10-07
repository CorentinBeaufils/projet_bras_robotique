#pragma once

#include <atomic>
#include <cstdint>

#include <QImage>
#include <QObject>

#include "perception_types.hpp"

// Capture + perception dans un thread SÉPARÉ de la GUI.
//
// Deux sens de communication :
//  - worker -> GUI : signaux (connexions queued). Les données sont copiées et
//    le slot s'exécute sur le thread GUI.
//  - GUI -> worker : PAS de slots queued (le worker tourne dans une boucle et ne
//    relit jamais sa file d'événements). À la place, des méthodes qui n'écrivent
//    QUE des std::atomic, appelées directement depuis le thread GUI.
class CameraWorker : public QObject {
    Q_OBJECT

public:
    explicit CameraWorker(int cam_index = 0, QObject* parent = nullptr);

    // --- Commandes appelables depuis N'IMPORTE QUEL thread (atomiques uniquement) ---
    void stop() { running_ = false; }
    void setPaused(bool paused) { paused_ = paused; }
    void triggerEmergencyStop() { emergency_stop_ = true; }  // verrouillé...
    void resetEmergencyStop() { emergency_stop_ = false; }   // ...jusqu'au réarmement
    void frameDisplayed() { frame_in_flight_ = false; }       // la GUI a fini l'image

public slots:
    void process();  // boucle capture + détection (lancée au démarrage du thread)

signals:
    // capture_ns : horodatage steady_clock de la capture, pour mesurer la latence.
    void frameReady(const QImage& image, qint64 capture_ns);
    void poseReady(const MarkerPose& pose);
    void telemetryReady(const Telemetry& telemetry);
    void finished();

private:
    int cam_index_;
    std::atomic<bool> running_{true};
    std::atomic<bool> paused_{false};
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> frame_in_flight_{false};  // une image est-elle en attente côté GUI ?
    std::uint64_t frames_dropped_ = 0;          // lu/écrit uniquement par le worker
};
