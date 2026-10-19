# Brief de projet — Bras robotique à suivi + anticipation

> **À lire par l'assistant qui reprend ce projet.** Ce document est autoportant : il
> contient tout le contexte nécessaire pour accompagner l'auteur du premier jalon
> jusqu'à la version finale, sans redemander le contexte. Traiter ce fichier comme la
> source de vérité du périmètre. Mettre à jour la section « État courant » au fil de l'eau.

---

## 0. Comment utiliser ce brief

- L'objectif du projet est autant **pédagogique et démonstratif** (se rendre employable)
  que fonctionnel. À chaque décision, privilégier ce qui **produit du code C++ qui vend**
  et ce qui est **finissable**, plutôt que l'exploit technique brut.
- Principe directeur permanent : **garder l'âme d'une idée, jeter sa partie "recherche".**
  Ici, l'âme = suivi + anticipation + cinématique + coordination. La partie jetée = la
  haute vitesse (attraper un objet lancé, hors de portée d'un amateur avec des servos).
- Escalade par jalons : **chaque phase doit produire une démo visible.** Ne jamais avoir
  un chantier béant pendant des semaines. Si une phase déborde, la découper, pas l'étaler.

---

## 1. Qui est l'auteur (contexte à connaître)

- Jeune ingénieur informatique fraîchement diplômé (**pas** d'école systèmes embarqués).
- Cible d'emploi : **développement C++**, embarqué et/ou robotique. Veut se démarquer.
- Peu codé en C++ pendant les études ; **en cours de remise à niveau C++ moderne** (a
  travaillé asio récemment). Bon niveau programmation, **débutant côté hardware** — n'a
  jamais construit de robot.
- Veut du **concret, manipuler, apprendre la cinématique** (n'en a pas peur).
- Rythme réaliste conseillé : **1 h à 2 h par jour, tous les jours**, régularité > intensité.
  Commit quotidien même minime. Chaque session démarre par un mini-objectif écrit.


### Projet précédent à réutiliser
L'auteur a déjà un dépôt **ESP32-CAM → UDP → PC** : protocole binaire maison sur UDP
(en-tête 30 o, fragmentation, CRC32, `frame_id`, timestamps), télémétrie (fps, pertes,
gigue), réassemblage, et une **étude comparée de récepteurs** (bloquant vs asio) avec
analyse de cause racine (`SO_RCVBUF`). **Ce protocole UDP + télémétrie + culture de la
latence se recyclent directement** comme canal de commande PC → microcontrôleur de ce
nouveau projet. C'est un point de continuité fort à exploiter, pas à réinventer.

---

## 2. Vision du projet

Un (puis deux) bras robotique(s) de table qui **suit(vent) un objet en mouvement lent**,
**prédit(sent) sa trajectoire** et va(vont) l'**intercepter** ; à terme, deux bras se
**passent** un objet (transfert posé/attrapé lent, pas un lancer).

Ce qui est intéressant et qu'on garde : **cinématique inverse, tracking visuel,
prédiction de trajectoire, planification, coordination inter-bras.** Tout cela est **ton
code**, pas celui d'un firmware tiers — c'est précisément ce qui a de la valeur pour un
recruteur C++/robotique.

---

## 3. Périmètre (à respecter strictement)

### DANS le périmètre
- Objets en mouvement **lent** : balle qui roule sur la table, objet sur plateau tournant,
  pendule lent.
- Environnement **sur table**, fixe, éclairage maîtrisé.
- Perception par **marqueur fiduciaire (ArUco)** d'abord : robuste, gratuit, pas de ML.
- Caméra = **webcam PC** (intégrée au début, puis USB sur trépied). **Pas** l'ESP32-CAM :
  ici la caméra est sur le PC, l'ESP n'est plus jamais un capteur vidéo (voir §7).
- **Cinématique directe et inverse** d'un bras à faible nombre d'axes (3 à 5 DOF).
- **Prédiction** de trajectoire par ajustement + extrapolation d'un modèle simple.
- Architecture **PC (perception + cinématique + planif) → microcontrôleur (exécution
  moteur + télémétrie)**, canal de commande réutilisant le protocole du projet précédent.
- **IHM de supervision en Qt** (composant assumé, pas optionnel) : flux caméra + détection
  superposée, position 3D et trajectoire prédite, état des axes, télémétrie/latence,
  contrôles start/stop + **arrêt d'urgence**. À construire **après** que la perception
  fonctionne, jamais avant (voir §6 et §7).
- Discipline d'ingénierie C++ : CMake, tests, CI, sanitizers (comme le projet précédent).

### HORS périmètre (au moins pour les premières versions)
- **Attraper un objet lancé / haute vitesse** — niveau labo, servos incapables. Exclu.
- **Perception markerless par deep learning** — plus tard éventuellement, pas au départ.
- Contrôle **sous la milliseconde**, retour de couple, dynamique avancée.
- Base mobile, vol, extérieur.
- (Décision ouverte) ROS 2 / MoveIt : optionnel, envisageable en montée en gamme, pas requis pour la v1.

---

## 4. Stack technique

**Langage & outillage (cœur, en C++ pour l'employabilité)**
- **C++20**, **CMake**, dépendances via vcpkg ou Conan.
- Tests **Catch2** ou **GoogleTest** ; **CI GitHub Actions** ; **ASan/UBSan** ; clang-tidy.
  (Reproduire la rigueur du projet ESP32 : lib pure testée séparée de l'I/O.)

**Mathématiques / cinématique**
- **Eigen** (algèbre linéaire, transformations homogènes, jacobienne).

**Vision**
- **OpenCV (API C++)**, module **aruco** pour la détection de marqueurs.
- Caméra : **webcam intégrée du PC pour démarrer** (gratuite, suffisante pour ArUco et le
  mouvement lent à 30 fps), puis **webcam USB posable sur trépied** quand le besoin de
  *placement* apparaît (Phase 2 : la caméra doit voir la zone de travail du bras, pas le
  visage de l'auteur). Une webcam grand public à ~60-80 € suffit ; **pas besoin** de
  caméra industrielle. Contrairement aux moteurs, la caméra n'est **pas** un achat qu'on
  repousse indéfiniment — mais l'intégrée couvre gratuitement les premières phases.
- **Calibration caméra obligatoire** (intrinsèques) via mire échiquier, pour transformer
  une détection ArUco en position 3D métrique juste. ⚠️ **Recalibrer à chaque changement
  de caméra** (les intrinsèques sont propres à chaque capteur) — une raison de plus de ne
  pas multiplier les caméras.
- Note framerate : le manque de cadence se compense **en logiciel par la prédiction**
  (Phase 4), pas en empilant des caméras. L'entrelacement temporel multi-caméras est un
  vrai sujet de recherche (synchro matérielle + reprojection dépendante de la profondeur)
  et **hors périmètre**. Si une 2e caméra est ajoutée un jour, c'est pour la **3D/stéréo**,
  pas pour le framerate.

**Actionnement**
- Microcontrôleur pour piloter les moteurs : **ESP32 ou STM32** (décision ouverte),
  firmware **PlatformIO**. Reçoit des consignes d'angles/positions, renvoie de la
  télémétrie. **Canal PC↔MCU = protocole UDP/série réutilisé du projet précédent.**

**Interface / supervision**
- **Qt (C++)** : IHM de supervision, **composant assumé du projet** (Qt est demandé sur le
  marché, et ici c'est la façade d'un vrai système temps réel, pas un tutoriel).
- Vraie difficulté C++ à maîtriser : le **handoff thread-safe entre le thread de
  perception/réseau et le thread GUI** (signals/slots inter-threads, ne jamais bloquer le
  fil qui dessine). Même famille que le handoff `LatestFrame` du projet précédent.
- ⚠️ **Ne pas commencer par l'IHM.** Elle habille un cœur qui fonctionne déjà : d'abord la
  perception (Phase 1, affichage minimal OpenCV suffit), Qt ensuite.

**Simulation (avant tout achat)**
- Option légère : **PyBullet** (Python) pour valider vite IK + trajectoire, OU une
  visualisation 3D maison en C++. But : voir le bras bouger **sans matériel**.
- Montée en gamme éventuelle : **ROS 2 + MoveIt** (bonus mots-clés emploi), plus tard.

**Matériel (À N'ACHETER QUE PLUS TARD — voir §7)**
- Bras : **reprendre un design open source** (imprimable / à base de servos ou steppers),
  ne pas concevoir la mécanique soi-même.
- Actionneurs : servos hobby (simples, jitter, pas de retour) vs servos série intelligents
  type Dynamixel (retour de position, chers) vs moteurs pas-à-pas + drivers (précis, plus
  complexes). **Décision différée jusqu'à validation en simu.**
- (La caméra n'est PAS dans cette liste différée : voir la section **Vision** ci-dessus —
  intégrée d'abord, webcam USB dès la Phase 2. La perception tourne sur le PC, jamais sur le MCU.)

---

## 5. Concepts à apprendre (jalonnés avec le plan)

- **Cinématique directe (FK)** : angles articulaires → position de l'effecteur. Formalisme
  **Denavit-Hartenberg** pour standardiser.
- **Cinématique inverse (IK)** : position visée → angles articulaires. Analytique possible
  pour peu d'axes ; sinon **numérique via la jacobienne**.
- **Jacobienne** : relie vitesses articulaires et vitesse de l'effecteur (IK numérique,
  contrôle en vitesse).
- **Calibration caméra** (intrinsèques) puis **calibration main-œil (hand-eye)** :
  transformation entre le repère caméra et le repère base du bras — indispensable pour
  commander le bras vers un point *vu* par la caméra.
- **Détection ArUco** : pose 6 DOF d'un marqueur, robuste et gratuite.
- **Prédiction de trajectoire** : ajuster un modèle (linéaire / parabolique / décélération
  ~constante pour une bille qui roule) sur les positions passées, extrapoler à l'instant
  d'interception.
- **Boucle temps réel** PC↔MCU : latence, cadence, sécurité (que fait le bras si le lien
  tombe ? → failsafe).
- **Handoff thread-safe perception → GUI (Qt)** : passer les données du thread de
  traitement au thread graphique via signals/slots inter-threads, sans blocage ni copie
  incohérente. Question classique d'entretien Qt.

---

## 6. Plan par phases (chaque phase = une démo)

### Phase 0 — Socle logiciel, zéro matériel
- Mettre en place le dépôt (CMake, CI, tests) à la manière du projet précédent.
- Écrire une **petite lib C++ de FK/IK** (Eigen) pour un bras à 3–4 DOF.
- **Visualiser** un bras simulé qui atteint une cible (simu ou viz maison).
- *Démo* : entrer une position cible, voir les angles calculés et le bras (virtuel) l'atteindre.

### Phase 1 — Perception seule, zéro moteur ⟵ **PREMIER JALON**
- Calibrer la webcam **intégrée** ; détecter un **marqueur ArUco** ; estimer et **afficher
  sa position 3D**. Affichage **minimal** (fenêtre OpenCV brute) — pas encore de Qt.
- *Démo* : bouger le marqueur à la main, lire sa position 3D en direct à l'écran.
- **C'est le tout premier objectif fixé à l'auteur.** S'il tient, le socle de tout le reste est là.

### Phase 1.5 — IHM Qt de supervision
- Remplacer l'affichage brut par une **interface Qt** : flux + détection superposée,
  position 3D live, télémétrie. Mettre en place le **handoff thread-safe perception → GUI**.
- Se fait **une fois la perception fonctionnelle**, pas avant. S'enrichit ensuite à chaque
  phase (trajectoire prédite, état des axes, arrêt d'urgence).
- *Démo* : la même détection qu'en Phase 1, mais dans une vraie IHM.

### Phase 2 — Un bras physique, cible statique
- (À ce stade, passer à la **webcam USB sur trépied** pour voir la zone de travail du bras,
  et **recalibrer**.)
- Assembler le bras (design open source), écrire le firmware MCU (exécute des angles).
- Établir le **canal PC→MCU** (réutiliser le protocole UDP/télémétrie du projet ESP32).
- **Hand-eye calibration** : mapper un point vu par la caméra vers le repère du bras.
- *Démo* : poser le marqueur immobile, le bras pointe/va dessus.

### Phase 3 — Suivi temps réel
- Le bras **suit** le marqueur qui bouge **lentement**, en boucle fermée.
- Gérer latence, lissage, limites articulaires, failsafe si perte de cible/lien.
- *Démo* : déplacer lentement l'objet, le bras le suit.

### Phase 4 — Anticipation
- **Prédire** la trajectoire d'un objet qui roule et **l'intercepter** (aller là où il
  *sera*, pas là où il est).
- *Démo* : lancer doucement la bille, le bras l'intercepte au bon endroit.

### Phase 5 — Deuxième bras + la passe (extension)
- Ajouter un second bras, **coordination**, transfert lent d'un objet de l'un à l'autre.
- *Démo* : les deux bras se passent l'objet.

> Extensions ultérieures possibles (même plateforme, juste du logiciel) : pseudo-rangement
> / tri d'objets, perception markerless, portage ROS 2 + MoveIt.

---

## 7. Garde-fous (règles de conduite du projet)

1. **Un seul bras d'abord.** Le 2e bras et la passe = phase 5, pas avant.
2. **Ne rien acheter tant que la simu n'a pas validé le besoin.** Le hardware ajoute tous
   les problèmes à la fois ; la simu isole le logiciel et est gratuite. Périmètre écrit →
   simulation → achat en dernier, et seulement ce que la simu a prouvé nécessaire.
3. **Ne pas concevoir la mécanique.** Reprendre un design de bras open source ; concentrer
   l'effort sur le logiciel (la force de l'auteur, et ce qui le vend).
4. **La perception tourne sur le PC**, pas sur le microcontrôleur. Le MCU exécute + remonte
   de la télémétrie. (⚠️ ne pas retomber dans le piège "vidéo sur l'ESP" du projet
   précédent : l'ESP était trop lent pour ça — ici il est actionneur, pas caméra.)
5. **Chaque phase finit par une démo + quelques lignes d'explication** (README/GIF). Tagger
   des jalons. Soigner la vitrine du dépôt (description, topics, badge CI valide, release).
6. **Discipline C++** identique au projet précédent : lib logique pure et testée, séparée
   de l'I/O et du matériel, pour pouvoir tester hors cible.

---

## 8. Décisions ouvertes à trancher tôt (avec l'auteur)

- Nombre d'axes du premier bras (suggestion : commencer à **3–4 DOF**).
- Type d'actionneur : servos hobby vs servos série (Dynamixel-like) vs pas-à-pas.
- Microcontrôleur du contrôleur d'axes : **ESP32** (réutilise l'écosystème connu) vs
  **STM32** (occasion d'apprendre le bas niveau / registres, compense l'absence d'école
  embarquée).
- Simulateur : **PyBullet** (rapide) vs viz C++ maison (tout en C++, cohérence).
- Adopter **ROS 2** dès le début ou seulement en montée en gamme.
- Langage de la couche perception/prototypage : tout **C++** (signal emploi) ou Python
  toléré pour le bac à sable de simu.

---

## 9. État courant du projet

> *(À mettre à jour à chaque session.)*

- **Phase en cours** : Phase 2 (un bras physique, cible statique) — **étape 2.0 : solder la
  dette**. Phases 0, 1 et 1.5 terminées et taguées **`v0.2.0`**.
- **Fait** :
  - **Phase 0** : lib `kinematics` C++20 — FK (bras plan N segments, matrices homogènes /
    Eigen) + IK analytique 2 segments (loi des cosinus + atan2, coude haut/bas, couronne
    atteignable via `std::optional`) ; tests Catch2 (IK testée par aller-retour FK∘IK),
    `-Werror`, ASan/UBSan ; démo `arm_demo` (SVG animé) + GIF dans le README.
  - **Phase 1** : calibration webcam intégrée (mire 9×6, `cornerSubPix`, reprojection
    0,19 px) ; détection ArUco ; pose 3D par `solvePnP` (`IPPE_SQUARE`), distance validée
    au réglet à 30 et 60 cm. Bug d'échelle diagnostiqué et corrigé : la taille du marqueur
    est le **côté extérieur complet** du carré noir.
  - **Lib `perception`** : `load_intrinsics`, `estimate_marker_pose`, `MarkerDetector`
    (détection + pose, observations sans pose conservées). 7 tests dont un `detect()` sur
    image synthétique (z attendu = f·L/s) ; tests validés par mutation (3 sabotages
    détectés). `pose_estimate` et `qt_supervisor` l'utilisent : plus aucun `solvePnP` ni
    lecture de YAML dans `app/`.
  - **Phase 1.5** : IHM Qt — worker de perception dans un `QThread`, signaux *queued*
    worker → GUI (copie profonde `cv::Mat` → `QImage`, `qRegisterMetaType`), commandes
    GUI → worker par `std::atomic` (pause, arrêt d'urgence verrouillé + réarmement),
    « une seule image en vol » par `atomic::exchange` : latence bornée à 58 ms contre
    1,7 s et croissante sans la garde (mesuré avec `--slow-gui`).
  - **Robustesse IHM** : intrinsèques chargées dans `main()` et injectées dans le worker ;
    erreur caméra remontée par signal → boîte de dialogue → sortie code 1 (*fail-safe*) ;
    thread et worker en **RAII** sur la pile (corrige un *use-after-free* trouvé par ASan
    sur le chemin « caméra absente », dû à des `deleteLater` prématurés).
  - Outillage : OpenCV 4.10 (API `ArucoDetector`), `.clang-format` + cibles `format` /
    `format-check`, `-Wpedantic -Werror` sur **toutes** les cibles, git, CI GitHub Actions
    (build Debug + ASan/UBSan + vérification du formatage).
- **Corrections d'historique** : le tag `v0.1.0` annoncé en Phase 0 n'a jamais existé (pas
  de git à l'époque) ; `v0.2.0` est le premier tag réel.
- **Étape 2.0 — dette à solder avant de toucher au matériel** :
  - CI : image avec OpenCV ≥ 4.7 + Qt 6 pour compiler et tester `perception` (aujourd'hui
    ignorée en CI) ; exiger `find_package(OpenCV 4.7)` pour qu'un OpenCV trop ancien soit
    ignoré au lieu de casser le build ;
  - lib `perception_viz` : `draw_observations(...)`, aujourd'hui copiée dans
    `pose_estimate` et `qt_supervisor` ;
  - configuration de scène (`config/scene.yml` ou argument) pour la taille du marqueur,
    aujourd'hui en dur dans deux fichiers — **pas** dans `camera_intrinsics.yml`, réécrit à
    chaque calibration ;
  - `.clang-tidy` + cible `tidy` (règles `bugprone`, `cppcoreguidelines`, `modernize`,
    `performance`, `readability`) ;
  - argument `--cam N` pour `qt_supervisor` (tester le chemin d'erreur sans modifier le
    code) ;
  - IHM : vidéo redimensionnable en gardant le ratio (plus de taille fixe 640×480).
- **Exigences de sûreté pour la Phase 2** (issues de la revue de la Phase 1.5) :
  - **arrêt d'urgence matériel direct** (coupure de l'alimentation des actionneurs) qui ne
    passe ni par le PC ni par l'application ; l'arrêt logiciel de l'IHM le complète, il ne
    le remplace pas ;
  - l'arrêt d'urgence logiciel ne doit **pas** vivre dans le worker caméra : il doit
    fonctionner même si la perception est morte ;
  - la commande du bras ne dépend pas de l'affichage : pose publiée à chaque image dans
    une « boîte aux lettres » (dernière valeur), horodatée ; au-delà d'un âge maximal, la
    pose est périmée et le bras s'arrête (*watchdog*, aussi côté MCU sur perte du lien).
- **Prochain pas concret** : étape 2.0 (dette), puis **2.1 — cinématique 3D** : base
  tournante (lacet = `atan2(y, x)`) + chaîne plane existante, FK/IK 3D testées par
  aller-retour, puis simulation d'un design de bras open source avec ses vraies longueurs.
  Aucun achat avant que la simulation ait validé l'espace de travail.
- **Décisions prises** : caméra = webcam PC (intégrée puis USB trépied), pas l'ESP32-CAM ;
  IHM en Qt ; viz C++ maison ; vcpkg pour Eigen et Catch2 ; **OpenCV ≥ 4.7 et Qt 6 par le
  système** ; pose par `solvePnP` ; commandes GUI → worker par atomiques ; politique
  « dernière image gagne » pour l'affichage ; dessin hors des libs de calcul ; IHM fermée
  sur erreur fatale (*fail-safe*).
- **Décisions ouvertes (Phase 2)** : design de bras open source ; actionneurs (servos
  hobby / servos série à retour de position / pas-à-pas) ; MCU (ESP32 ou STM32) ; liaison
  PC ↔ MCU (UDP Wi-Fi ou série USB) ; budget.
- **Blocages** : —

---

## 10. Définition de "fini"

Un système (idéalement deux bras, au minimum un) qui **suit un objet lent, prédit sa
trajectoire et l'intercepte / le transmet**, avec :
- le code C++ propre (tests, CI, sanitizers), lib cinématique réutilisable ;
- le canal PC↔MCU dérivé du protocole du projet précédent ;
- un **README avec GIF de démonstration** et un court écrit expliquant l'IK et la
  prédiction ;
- une **release taggée**.
À ce stade : s'arrêter, capitaliser, et seulement ensuite envisager le drone (en
simulation d'abord).
