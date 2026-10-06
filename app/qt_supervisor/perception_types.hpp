#pragma once

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

// Permet à MarkerPose de traverser une connexion signals/slots *queued*
// (indispensable dès qu'on l'envoie du thread worker au thread GUI).
Q_DECLARE_METATYPE(MarkerPose)
