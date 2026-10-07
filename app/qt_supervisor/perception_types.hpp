#pragma once

#include <cstdint>

#include <QMetaType>

// Pose d'un marqueur dans le repère caméra, en mètres.
// visible = false quand aucun marqueur n'est détecté sur l'image courante.
struct MarkerPose {
    bool visible = false;
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Télémétrie du worker, émise environ une fois par seconde.
struct Telemetry {
    double fps = 0.0;                  // images traitées par seconde
    double processing_ms = 0.0;        // temps moyen détection + pose, par image
    std::uint64_t frames_dropped = 0;  // images jetées car la GUI n'avait pas fini
    bool paused = false;
    bool emergency_stop = false;
};

// Permet à ces types de traverser une connexion signals/slots *queued*
// (indispensable dès qu'on les envoie du thread worker au thread GUI).
Q_DECLARE_METATYPE(MarkerPose)
Q_DECLARE_METATYPE(Telemetry)
