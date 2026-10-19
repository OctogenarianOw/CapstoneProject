# Prompt maître, spécification complète du projet

**Geometry-Aware UAV Autonomous Navigation for Safe Landing Detection**
UTS, Capstone A/B, Dorian Veloso, superviseur Dr. Graeme Best, soutenance décembre 2026.
Version du 17 octobre 2026.

Ce document dit **ce que le projet doit faire dans sa totalité**. Il ne dit pas où en est le code : l'état d'avancement vit dans `claude/etat-du-projet-uav.md`, qui est le document de passation et qui bouge à chaque session. Les deux se lisent ensemble : celui-ci est la cible, l'autre est la position.

---

## 0. Instructions à l'assistant

Tu reprends un projet de recherche de master qui court depuis mars 2026 et qui doit être soutenu en décembre 2026. Le code sera défendu devant un jury par son auteur, pas par toi.

**Mode de travail, révisé le 17 octobre 2026.** Dorian t'autorise à construire sans expliquer au préalable et à ne pas attendre sa validation ligne à ligne. Il relit derrière. Cette autorisation porte sur le **rythme**, pas sur la traçabilité : la justification de chaque choix doit continuer d'exister, simplement elle est **écrite** au lieu d'être échangée.

**Ce que tu fais :**

1. Tu construis directement, puis tu rends compte : ce que tu as écrit, pourquoi ce choix plutôt qu'un autre, ce que ça mesure. Chaque module porte en tête un commentaire en anglais de cinq à dix lignes indiquant son rôle, le choix de conception retenu, l'alternative écartée et la raison. C'est le matériau de la soutenance, et il doit s'accumuler tout seul pendant que le code avance.
2. **Une modification à la fois**, avec une mesure avant et une mesure après. Jamais deux changements simultanés : l'effet ne serait plus attribuable. Cette règle ne relève pas de la pédagogie mais de la validité des résultats, elle reste entière.
3. **Mesurer avant de corriger.** Une hypothèse non vérifiée ne justifie pas un correctif. Plusieurs hypothèses plausibles ont déjà été réfutées par la mesure dans ce projet.
4. Tu donnes **la séquence de commandes complète à chaque fois**, sans abréger, sans renvoyer à une étape précédente d'une conversation.
5. Tu vérifies les noms de topics avec `ros2 topic list` et les définitions de messages avec `ros2 interface show`, jamais de mémoire ni depuis un tutoriel.
6. Tu distingues toujours trois registres : ce qui est **mesuré**, ce qui est **supposé**, ce qui est **choisi**. Un seuil mesuré se défend, un seuil choisi se justifie, un seuil supposé se signale.
7. Code et commentaires en anglais. Conversation en français. Pas de tirets doubles ni de tirets cadratins dans les textes produits.

**Ce que tu ne fais pas :**

- Générer un stack complet d'un coup. Non plus pour des raisons de compréhension, mais parce qu'un ensemble livré en bloc ne peut pas être mesuré composant par composant, et qu'un projet qui ne mesure pas n'a pas de chapitre résultats. Un composant, sa mesure, sa conclusion, puis le suivant.
- Empiler des correctifs symptomatiques. On remonte à la cause.
- Partir dans plusieurs directions à la fois. Un fil, une mesure, une conclusion, puis le fil suivant.
- Présenter comme acquis un résultat non mesuré.

---

## 1. Cadre et périmètre

### 1.1 Ce qui est évalué dans un mémoire de master

Pas le volume de code. Quatre choses :

- une **question de recherche** nette, à laquelle on peut répondre par oui, par non ou par un chiffre,
- une **méthode** qui permet effectivement d'y répondre, et dont les choix sont justifiés,
- des **mesures** reproductibles, comparées à une référence, avec leur incertitude,
- une **honnêteté** sur ce qui n'a pas marché et sur les limites du travail.

Un résultat négatif documenté vaut mieux qu'un résultat positif non mesuré. Le projet en contient déjà un, décrit en 6.3, et il doit figurer dans le mémoire.

### 1.2 Périmètre

- **Simulation uniquement.** ROS 2 Jazzy, Gazebo Harmonic 8.11.0, PX4 SITL v1.17.0-alpha1, WSL2 Ubuntu 24.04. Aucun vol réel. Cette limite est assumée et déclarée, pas cachée.
- **Un seul drone.** Pas de flotte, pas de coordination multi-agents.
- **Localisation par GPS** via l'EKF2 de PX4, pas de SLAM. Décision prise sur suggestion du superviseur en semaine 2.
- **Pas de vent.** Gazebo Harmonic ne le simule pas par défaut, l'ajouter changerait le modèle d'énergie sans servir la question de recherche.
- **Capteur unique**, un LiDAR 3D orienté vers le bas. Pas de caméra, pas de fusion multi-capteurs.
- **Environnement statique.** Le terrain ne change pas pendant la mission. Le vieillissement probabiliste de la carte est une extension future identifiée, pas un objectif.

### 1.3 Infrastructure

| Élément | Valeur |
|---|---|
| Dépôt | github.com/OctogenarianOw/CapstoneProject |
| Branche de travail | `dronePropre` |
| Workspace actif | `~/ros2_ws/droneAfter` |
| Package ROS | `uav_core` |
| Archive de l'ancien stack | `~/ros2_ws/droneBefore`, tag `archive-pre-rebuild` |
| Rendu | toujours `HEADLESS=1`, bug `dxgkrnl` de WSL2 non résolu |
| Machine de démonstration finale | Linux natif, demandée au département |

---

## 2. Question de recherche et contributions

### 2.1 Question de recherche

> **How can a UAV autonomously search an unknown environment for safe landing zones, while ensuring continuous reachability of a geometrically confirmed safe site whenever one has been identified?**

La clause finale, *whenever one has been identified*, n'est pas une précaution de style. Elle résout le paradoxe d'amorçage : à t = 0, aucune zone sûre n'est connue, donc une garantie inconditionnelle de rejoignabilité serait impossible à tenir et la mission ne pourrait jamais démarrer. Voir S15.3 pour la politique d'amorçage retenue.

### 2.2 Décomposition en sous-questions mesurables

La question principale est trop large pour être mesurée directement. Elle se décompose en quatre sous-questions, chacune associée à une expérience et à un chiffre.

- **SQ1.** Avec quelle fidélité une carte d'élévation 2.5D construite en vol permet-elle de classer le terrain en sûr ou non sûr, comparée à une vérité terrain exacte ? *Mesure : recall, precision, faux acceptables dangereux, par zone.*
- **SQ2.** Un modèle d'énergie simple, calibré empiriquement en simulation, prédit-il la consommation d'une trajectoire avec une erreur suffisamment faible pour fonder une garantie de rejoignabilité ? *Mesure : erreur relative de prédiction sur des trajectoires connues.*
- **SQ3.** L'invariant de faisabilité énergétique (C1) empêche-t-il effectivement le drone d'entrer dans un état où plus aucune zone sûre connue n'est atteignable, et à quel coût en couverture explorée ? *Mesure : violations de l'invariant, surface couverte, comparaison avec et sans C1.*
- **SQ4.** La pondération par marge énergétique (C2) améliore-t-elle la qualité de la mission par rapport à une sélection de frontière purement informationnelle, et quel est l'effet du paramètre α ? *Mesure : balayage de α sur cinq valeurs, métriques de 5.3.*

### 2.3 Contribution C1, invariant de faisabilité énergétique

Une frontière candidate n'est **admissible** que si, une fois atteinte, le drone dispose encore de l'énergie nécessaire pour rejoindre la zone d'atterrissage sûre la plus proche du registre et s'y poser :

```
E_go(p_actuelle -> f) + E_return(f -> slz*) + E_land  <=  B_current - R
```

avec `slz*` la zone du registre minimisant `E_return` depuis `f`, `B_current` l'énergie restante estimée et `R` une réserve fixe.

Ce qui distingue cette formulation de la littérature existante sur l'exploration sous contrainte énergétique : `slz*` n'est **pas** une base de retour statique et prédéfinie. C'est une entrée d'un **registre dynamique**, construit en ligne par l'analyse de terrain, qui peut **grandir** quand de nouvelles zones sont confirmées et **rétrécir** quand une zone est invalidée par une observation ultérieure. L'invariant doit donc être réévalué à chaque mise à jour du registre, et pas seulement à chaque sélection de frontière. Le cas critique, spécifié en S9.5, est la disparition de la zone sur laquelle reposait la garantie alors que le drone est en route.

### 2.4 Contribution C2, utilité à marge énergétique

Parmi les frontières admissibles, le classement se fait par :

```
U(f) = alpha * G(f) + (1 - alpha) * M(f)
```

- `G(f)` : gain d'information normalisé, nombre estimé de cellules inconnues qui deviendraient observées depuis `f`, divisé par le maximum sur l'ensemble des candidates du cycle.
- `M(f)` : marge énergétique normalisée, `(B_current - R - E_total(f)) / (B_current - R)`, bornée à [0, 1].
- `alpha` dans [0, 1], paramètre d'étude. `alpha = 1` redonne une exploration purement informationnelle, `alpha = 0` une exploration purement conservatrice.

C2 n'est pas une simple pondération cosmétique : elle encode l'arbitrage central du problème, aller chercher de l'information là où elle est, ou rester près d'un refuge confirmé.

### 2.5 Ce qui n'est pas une contribution

À dire explicitement dans le mémoire, parce qu'un jury le demandera :

- la cartographie d'élévation 2.5D par filtre de Kalman scalaire par cellule : état de l'art, réimplémenté pour maîtriser la chaîne,
- l'analyse de traversabilité par pente, rugosité et marche : suit la méthodologie de Beycimen et al. (2025),
- la détection de frontières et le remplissage par diffusion : Yamauchi (1997) et descendants,
- A* : manuel.

La contribution est l'articulation entre un registre de zones sûres découvert en ligne et une garantie énergétique continue, plus la politique de sélection qui en découle.

---

## 3. Vue d'ensemble du système

### 3.1 Mission nominale, en une phrase

Un multirotor décolle d'un point connu dans un terrain inconnu, le cartographie progressivement avec un LiDAR orienté vers le bas, identifie et confirme géométriquement des zones où il pourrait se poser sans dommage, maintient à tout instant la certitude de pouvoir rejoindre au moins une de ces zones avec l'énergie restante, et termine sa mission en s'y posant.

### 3.2 Déroulé nominal, pas à pas

1. Armement, passage en mode offboard, décollage vertical jusqu'à l'altitude de croisière.
2. Le point de décollage est enregistré comme zone sûre provisoire, étiquetée comme telle (S15.3).
3. La cartographie démarre : chaque scan LiDAR est transformé en repère monde et fusionné dans la grille d'élévation.
4. L'analyse de terrain tourne en continu sur la carte et alimente la classification par cellule.
5. Les cellules sûres sont agrégées en zones ; les zones qui passent les critères d'empreinte et de confiance entrent au registre SLZ.
6. Les frontières sont détectées **sur la carte**, jamais sur le nuage brut.
7. Chaque frontière candidate est filtrée par C1, puis classée par C2.
8. La meilleure frontière admissible devient l'objectif ; un chemin est planifié et suivi.
9. À chaque mise à jour du registre ou de l'estimation d'énergie, l'invariant est réévalué. S'il est menacé, le drone interrompt l'exploration et engage un retour vers la meilleure zone atteignable.
10. La mission se termine par un des trois cas : couverture complète, budget énergétique atteignant le seuil d'engagement, ou commande d'arrêt. Dans tous les cas, le drone se pose sur une zone du registre, pas n'importe où.
11. Les données de la mission sont enregistrées pour rejeu et analyse hors ligne.

### 3.3 Architecture logicielle cible

Séparation nette entre les **modules sans ROS**, testables seuls, et les **nodes ROS**, qui ne font que du transport et de l'orchestration. Cette séparation est déjà en place et doit être tenue : elle est ce qui rend le projet testable sans simulateur.

Modules purs (`src/uav_core/uav_core/`, sans import `rclpy`) :

| Module | Rôle |
|---|---|
| `elevation_grid.py` | grille 2.5D, fusion par cellule |
| `terrain_analysis.py` | couches pente, rugosité, marche, validité |
| `slz_detection.py` | classification par cellule, empreinte, étiquetage de zones |
| `slz_registry.py` | registre dynamique, cycle de vie des zones *(à écrire)* |
| `energy_model.py` | prédiction de consommation, calibration *(à écrire)* |
| `frontier_detection.py` | frontières sur la carte, regroupement, gain d'information *(à écrire)* |
| `planner.py` | A* sur la grille d'obstacles, coût de chemin *(à écrire)* |
| `mission_policy.py` | C1, C2, sélection *(à écrire)* |

Nodes ROS :

| Node | Rôle |
|---|---|
| `offboard_demo.py` | test minimal de la chaîne de contrôle, conservé comme référence |
| `square_mission.py` | trajectoire géométrique simple, conservé |
| `survey_mission.py` | balayage en lignes parallèles, banc de test déterministe |
| `cloud_world.py` | ingestion LiDAR, changement de repère, fusion dans la grille |
| `exploration_manager.py` | machine à états, décision, seul détenteur de l'autorité de décision *(à écrire)* |
| `trajectory_executor.py` | suit un chemin, publie les consignes PX4 *(à écrire ou issu de survey_mission)* |

**Règle d'architecture à tenir :** le détecteur de frontières produit un catalogue, le gestionnaire d'exploration décide. Un seul composant détient l'autorité de décision. C'est ce qui permet d'intégrer C1 et C2 à un seul endroit et de les désactiver proprement pour les expériences de référence.

### 3.4 Invariants globaux

Ces propriétés doivent tenir à tout instant de la mission. Chacune est vérifiable dans les journaux, et leur vérification fait partie des résultats.

- **I1.** Si le registre SLZ est non vide, alors l'énergie restante est suffisante pour rejoindre au moins une de ses entrées et s'y poser, réserve comprise.
- **I2.** Aucune cellule n'est déclarée sûre si une cellule de sa fenêtre d'analyse n'a jamais été observée. Mieux vaut ne rien dire que déclarer plat un terrain non vu.
- **I3.** Le drone ne reçoit jamais de consigne l'amenant au dessus d'une cellule dont l'obstacle connu dépasse l'altitude de croisière.
- **I4.** Toute zone du registre a été observée depuis au moins deux passages distincts, avec une variance par cellule inférieure au seuil de confiance.
- **I5.** La commande d'atterrissage n'est jamais émise ailleurs que sur une zone confirmée du registre, sauf en cas d'urgence énergétique explicitement journalisée.

---

## 4. Spécification composant par composant

Chaque composant est spécifié par : rôle, entrées, sorties, paramètres, algorithme, critère d'acceptation, pièges.

### S1. Modèle de drone et capteur

**Rôle.** Fournir un drone simulé dont le capteur est réaliste au point que la fusion multi-observations ait un sens.

**Spécification.** Modèle `x500_slz`, inclut `model://x500` de PX4 et ajoute un `lidar_link`.

| Paramètre | Valeur | Justification |
|---|---|---|
| pose du capteur | `0 0 -0.15` sous `base_link` | à 5 cm, le LiDAR détectait les bras du drone ; résolu par la géométrie, pas par un filtre logiciel |
| angles verticaux | `-1.5708` à `0.0` | hémisphérique vers le bas ; le modèle PX4 d'origine montait à +30°, soit un quart des rayons vers le ciel |
| échantillons | 360 horizontaux, 32 verticaux | 11 520 points par scan, nuage organisé 32 x 360 |
| `update_rate` | 10 Hz | |
| `range` | 0.3 à 25 m | |
| `noise` | gaussien, écart type 0.02 m | sans bruit, la fusion multi-observations n'a aucun sens |
| `topic` | `lidar` | produit `/lidar/points` côté Gazebo |
| `gz_frame_id` | `lidar_link` | permet de vérifier qu'on lit le bon capteur |

**Critère d'acceptation.** Dans RViz, `Fixed Frame` sur `lidar_link`, le nuage forme une calotte régulière sous le drone, sans retour sur la cellule du drone lui même, et sans points au dessus de l'horizon.

**Pièges.** PX4 construit lui même le chemin du modèle et ignore `GZ_SIM_RESOURCE_PATH` : il faut un lien symbolique dans `~/PX4-Autopilot/Tools/simulation/gz/models/`. L'airframe `4030_gz_x500_slz` doit être copié dans la ROMFS et déclaré dans le `CMakeLists.txt` des airframes, sur une ligne indentée par une tabulation.

### S2. Monde de validation et vérité terrain

**Rôle.** Permettre de mesurer l'erreur de la carte et de la classification contre une référence exacte, pas contre une impression visuelle. C'est la pièce qui transforme le projet d'une démonstration en une expérience.

**Principe non négociable.** La géométrie du monde et la vérité terrain sont générées par `tools/make_world.py` depuis **la même fonction de hauteur analytique** `height_at(x, y)`. Elles ne peuvent donc pas diverger.

**Sorties.**
- `worlds/validation.sdf` : le `default.sdf` de PX4 avec le terrain injecté, monde renommé pour correspondre à `PX4_GZ_WORLD`.
- `worlds/validation_truth.npz` : clés `centres`, `truth`, `zone`, `resolution`, `zone_names`.

**Contenu requis.** Un monde de validation utile doit contenir, au minimum :

| Zone | Rôle expérimental |
|---|---|
| plat | plancher de bruit, référence de la métrique |
| pente acceptable, 3° | **cas limite positif**, doit être acceptée, teste que le seuil n'est pas trop strict |
| pente refusée, 10° | **cas limite négatif**, doit être refusée, teste la séparation |
| zone rugueuse | teste que rugosité et pente sont bien séparées par l'analyse |
| obstacle vertical, cube de 4 m | teste la détection de marche et le comportement aux discontinuités |

Contraintes de construction, apprises à la dure :

- **Géométrie pleine, jamais sur pilotis.** Une rampe modélisée par une plaque inclinée sur supports laisse le LiDAR voir le sol par dessous depuis les côtés, et ces retours tombent dans les mêmes cellules que la surface supérieure. Tant qu'un tel défaut existe, les faux acceptables autour des rampes ne sont pas interprétables.
- **Alignement sur la grille de vérité.** Les bords de zone doivent coïncider avec les bords de cellule de 1 m, sinon une cellule contient deux géométries et la comparaison est faussée.
- **Un seul lien statique** pour tout le terrain, pour éviter des centaines de corps à suivre pour le moteur physique.
- **Graine aléatoire fixée** pour la zone rugueuse, afin que le monde soit reproductible.
- Tags PX4 obligatoires dans le SDF : `<gravity>`, `<magnetic_field>`, `<atmosphere type="adiabatic"/>`, `<spherical_coordinates>`, et physique `ode` et non `dart`, sinon les contrôles de pré-vol échouent et le drone refuse de s'armer.

**Critère d'acceptation.** `tools/check_truth.py` valide la vérité terrain contre les seuils de détection **sans voler** : il doit confirmer que la pente à 3° est sous le seuil, celle à 10° au dessus, que la zone rugueuse dépasse le seuil de rugosité et que le cube dépasse le seuil de marche. Si la vérité terrain elle même ne sépare pas les cas, aucun résultat de vol n'est interprétable.

### S3. Interface PX4

**Rôle.** Armer, passer en offboard, envoyer des consignes de position, lire l'état du véhicule, et ne jamais perdre le lien offboard.

**Entrées lues.**

| Topic | Usage |
|---|---|
| `/fmu/out/vehicle_local_position_v1` | position NED, validité `xy_valid` et `z_valid`, incertitude `eph` |
| `/fmu/out/vehicle_attitude` | quaternion FRD vers NED, ordre w, x, y, z |
| `/fmu/out/vehicle_status_v4` | mode de vol, état d'armement, `nav_state` |
| `/fmu/out/battery_status_v1` | tension, courant, `remaining`, `discharged_mah` |
| `/fmu/out/timesync_status` | décalage d'horloge, 1 Hz |
| `/fmu/out/home_position_v1` | point de décollage |

**Sorties.** `OffboardControlMode` et `TrajectorySetpoint` à 10 Hz minimum, `VehicleCommand` pour armement, mode et atterrissage.

**Règles obligatoires.**

1. Les champs non utilisés de `TrajectorySetpoint` (`velocity`, `acceleration`, `jerk`, `yawspeed`) doivent valoir **NaN**, pas zéro. Zéro est une consigne, pas une absence de consigne. PX4 reçoit alors des consignes contradictoires et met `accepts_offboard_setpoints: false`, sans message d'erreur.
2. Le champ `timestamp` est un **temps depuis le démarrage du système, en microsecondes**. Envoyer un temps Unix place le message 56 000 ans dans le futur.
3. `timesync_status` n'arrive qu'à 1 Hz. Recopier son `timestamp` tel quel date dix consignes consécutives à la même valeur, PX4 juge le flux périmé, lève `offboard_control_signal_lost` et déclenche un retour automatique. Il faut mémoriser le **décalage** entre horloge PX4 et horloge locale, puis extrapoler :

```python
def on_timesync(self, msg):
    local_us = self.get_clock().now().nanoseconds // 1000
    self.px4_offset_us = msg.timestamp - local_us

def px4_now(self):
    if self.px4_offset_us is None:
        return 0
    return int(self.get_clock().now().nanoseconds // 1000 + self.px4_offset_us)
```

4. Les topics PX4 sont **versionnés**. S'abonner à un topic inexistant ne produit aucune erreur, juste un silence.
5. QoS PX4 : `BEST_EFFORT`, `TRANSIENT_LOCAL`, `KEEP_LAST`, depth 1. QoS LiDAR : `qos_profile_sensor_data`. **Réutiliser le profil PX4 pour le LiDAR ne produit aucun message et aucune erreur.**
6. Attendre `xy_valid` et `z_valid` avant d'entamer la séquence d'armement, sinon condition de course au démarrage.

**Critère d'acceptation.** Un vol de 5 minutes en offboard continu, sans perte de lien, sans basculement en mode de secours, journalisé.

### S4. Exécuteur de trajectoire

**Rôle.** Transformer une liste de points de passage ou un chemin planifié en consignes PX4, et signaler l'arrivée.

**Paramètres actuels du banc de test `survey_mission.py` :**

| Paramètre | Défaut | Rôle |
|---|---|---|
| `altitude` | 8.0 m | 4 m au dessus du relief le plus haut du monde de validation, le cube de 4 m (vérité terrain, maximum 4.0 m) |
| `line_length` | 50.0 m | couvre les zones à ±20 m |
| `line_spacing` | 15.0 m | avec 20 m de portée utile, les bandes se recouvrent largement |
| `n_lines` | 4 | couvre 45 m |
| `accept_radius_xy` | 1.5 m | au dessus de l'incertitude de pose, `eph` autour de 0.15 m |
| `accept_radius_z` | 0.8 m | PX4 régule bien mieux l'altitude que la position |

**Règle.** Les seuils d'arrivée horizontal et vertical sont **séparés**. Une distance 3D unique bloquerait la validation sur une petite erreur verticale alors que le drone est pile au dessus du point.

**Exigence pour la phase autonome.** L'exécuteur doit accepter un chemin reçu dynamiquement, pouvoir être préempté en cours de segment, et publier son état d'avancement. Il doit redéfinir l'arrivée comme la distance euclidienne réelle à l'objectif final, pas comme l'achèvement d'un segment partiel : confondre les deux est l'origine d'un bug de l'ancienne architecture où le drone se déclarait arrivé sans l'être.

**Critère d'acceptation.** Le survey de 9 points de passage s'exécute en environ 45 secondes, reproductible, sans dérive de l'altitude au delà de 0.8 m.

### S5. Carte d'élévation 2.5D

**Rôle.** Transformer un flux de nuages bruités en une estimation d'élévation par cellule, avec son incertitude. C'est le composant sur lequel repose tout le reste : le superviseur a explicitement indiqué que l'analyse de terrain doit être calculée sur une carte et non sur le nuage brut, qui est essentiellement du bruit.

**Pipeline, dans l'ordre exact.**

1. **Décodage du `PointCloud2`** en lisant les décalages dans `msg.fields`, jamais en les codant en dur. Un `.copy()` est obligatoire avant `.view(np.float32)` : une tranche de tableau numpy n'est pas contiguë.
2. **Filtrage sur la norme** du vecteur, entre `min_range` et `max_usable_range`. Gazebo encode les rayons sans retour par des **zéros**, pas par des `inf` : un test de finitude ne les élimine pas.
3. **Changement de repère**, en trois temps : FLU capteur vers FRD corps par `pts * [1, -1, -1]`, puis ajout du décalage capteur, puis rotation par la matrice issue du quaternion d'attitude, puis ajout de la position NED. **Sans cette rotation, un point à 15 m devant est placé à la mauvaise altitude dès que le drone s'incline.** C'était le bug qui faisait virer tout le terrain au rouge dans l'ancienne architecture.
4. **Variance par point** : `sigma = sigma_range + sigma_angular * distance`, puis `variance = sigma ** 2`.
5. **Fusion** dans la grille, filtre de Kalman scalaire par cellule :

```
k = p / (p + r)
elevation += k * (mesure - elevation)
variance = max((1 - k) * p, min_sigma_m ** 2)
```

6. **Sauvegarde périodique** toutes les 5 secondes, par écriture dans un fichier temporaire puis `os.replace`, qui est atomique.

**Paramètres.**

| Paramètre | Défaut |
|---|---|
| `min_range` | 0.3 m |
| `max_usable_range` | 20.0 m |
| `sensor_offset_down` | 0.15 m |
| `grid_size` | 100.0 m |
| `grid_resolution` | 1.0 m |
| `sigma_range_m` | 0.02 |
| `sigma_angular_rad` | 0.009 |
| `min_sigma_m` | 0.02 |
| `map_path` | `/tmp/elevation_map.npz` |

**Le plancher de variance `min_sigma_m` n'est pas cosmétique.** Sans lui, la variance d'une cellule décroît indéfiniment : après 500 observations le gain vaut 0.002, et une mesure écartée de 10 cm déplace l'estimation de 0.2 mm. La cellule est **gelée** et ne peut plus se corriger, même par une bien meilleure observation prise de près. Une cellule sous le drone posé avait accumulé 120 000 observations avec une confiance absurde. Ce gel produisait des tranchées fantômes de 4 m de profondeur aux bords de rampe.

**Exigence restante : bruit de processus.** Le plancher de variance traite le symptôme. Un terme de bruit de processus, qui fait croître la variance avec le temps écoulé ou la distance parcourue depuis la dernière observation, est la formulation propre et doit être évalué contre le plancher seul.

**Critère d'acceptation.** Erreur RMS d'élévation sur sol plat inférieure à 0.25 m, avec couverture de 100 % des cellules survolées, mesurée par `tools/validate_map.py` contre la vérité terrain. Référence actuelle : 0.240 m.

**Pièges.** `/tmp/elevation_map.npz` est écrasé par n'importe quel lancement ultérieur du node, même sans vol. Copier immédiatement la carte sous `data_survey_vN.npz` après chaque vol. Une session de validation a déjà été perdue ainsi.

### S6. Couche d'état d'observation

**Rôle.** Savoir ce qui est connu, ce qui ne l'est pas, et avec quelle confiance. Sans cette couche, ni les frontières ni la validité de l'analyse ne sont définissables.

**Trois tableaux par cellule, déjà présents :** `elevation`, `variance`, `count`.

**Exigence.** Une cellule est classée dans un des états suivants, et cet état doit être exposé comme une couche à part entière :

| État | Condition |
|---|---|
| `UNKNOWN` | `count == 0` |
| `SPARSE` | `0 < count < n_min` ou `variance > var_max` |
| `OBSERVED` | `count >= n_min` et `variance <= var_max` |

`n_min` et `var_max` sont des paramètres à fixer par mesure, pas par intuition. Les deux seuils d'observation ne sont pas nécessairement les mêmes pour la détection de frontières et pour l'analyse géométrique : dans l'ancienne architecture ils valaient 3 et 5 respectivement, appliqués indépendamment, et c'était un choix délibéré à redocumenter.

### S7. Analyse de terrain

**Rôle.** Produire, depuis la carte, les trois descripteurs géométriques qui définissent la sûreté d'un atterrissage, plus un masque de validité.

**Signature.** `analyse(elevation, count, resolution, radius=1)`, sans ROS. Retourne `slope`, `roughness`, `step`, `valid`.

| Couche | Définition | Justification du choix |
|---|---|---|
| `slope` | inclinaison du plan des moindres carrés ajusté sur la fenêtre | **pas une différence finie** : avec 5 cm de bruit entre deux cellules voisines, une différence finie produit 6° de fausse pente ; l'ajustement de plan divise cette sensibilité par trois environ |
| `roughness` | écart type des résidus au plan | **indépendante de la pente par construction**, puisque le plan est retiré avant de mesurer la dispersion ; une rampe lisse donne pente forte et rugosité faible |
| `step` | écart maximal entre la cellule et ses voisines | capte ce que les deux autres manquent, un rebord vertical entre deux zones par ailleurs planes |
| `valid` | faux dès qu'une cellule de la fenêtre n'a jamais été observée | décision de sécurité, invariant I2 |

Implémentation vectorisée par empilement de copies décalées de la grille, avec formes closes pour les coefficients du plan, la grille étant régulière.

**Critère d'acceptation, déjà atteint.** L'analyse sépare correctement pente et rugosité : sur la carte de validation, la rampe lisse à 10° donne une rugosité de 0.006 m et la zone rugueuse 0.011 m contre 0.012 m en vérité. C'était la question ouverte de cette étape, et la réponse est oui.

**Mesure à conserver dans le mémoire : le plancher de bruit.** Sur le plat intérieur, la distribution de pente mesurée donne p50 = 1.14°, p90 = 3.06°, p95 = 5.41°, p99 = 40.44°. Le saut entre p95 et p99 signale une population distincte de cellules aberrantes, environ 1 %, concentrées autour des structures. Cette mesure est ce qui **justifie** le seuil de pente, voir S8.

### S8. Détection des zones d'atterrissage

**Rôle.** Passer de trois couches continues à une liste de zones nommées, chacune caractérisée.

**Étages, volontairement séparés.**

1. `classify_cells(layers, max_slope_deg, max_roughness_m, max_step_m, variance=None, max_variance=None)` : test par cellule. Le gate de variance existe dans la signature mais n'est pas encore utilisé ; il doit l'être, c'est le lien avec I4.
2. `dilate(mask, radius)` : grossit un masque, pour la marge de sécurité.
3. `footprint_ok(cell_ok, radius)` : ne garde que les cellules dont toute l'empreinte du drone passe le test.
4. `label_zones(candidate)` : regroupement par remplissage par diffusion en 8-connexité, écrit à la main pour ne dépendre que de numpy.
5. `zone_summary(...)` : un enregistrement par zone, trié par aire.
6. `detect(...)` : enchaîne le tout.

**Seuils de classification, mesurés et non choisis.**

```
pente     <= 6.0 degres
rugosite  <= 0.05 m
marche    <= 0.24 m
```

Le seuil de pente de 6° vient d'une mesure, pas d'une convention : le plat atteint p95 à 5.41° et la pente de 10° commence à p01 à 6.96°. Il existe donc un seuil qui sépare le plat de la pente de 10° pour 95 % des cellules. Le monde de validation contient ensuite une pente de 3° précisément pour vérifier que ce seuil n'est pas trop strict dans l'autre sens : elle est acceptée entre 84 % et 98 % des cellules selon les versions de carte, ce qui valide le seuil des deux côtés.

**Calcul du rayon d'empreinte.**

```python
radius = int(max(0.0, (footprint_m / resolution - 1.0) / 2.0))
```

L'ancienne formule `max(1, int(round(footprint_m / resolution / 2.0)))` avait deux défauts : `round(1.5)` vaut 2 en Python, arrondi au pair, et le `max(1, ...)` interdisait toute empreinte inférieure à 3 cellules.

**Élément structurant, tranché le 17 octobre 2026.** L'élément d'empreinte doit être une **sur-approximation** du disque, c'est à dire l'ensemble des cellules que le disque d'empreinte recouvre, même partiellement. Toute autre convention accepte des cellules qu'elle n'a pas vérifiées, ce qui est inacceptable pour un filtre de sécurité.

Deux conventions ont été écartées :

- l'ancien rayon `int(max(0.0, (footprint_m / resolution - 1.0) / 2.0))` retient le plus grand carré de cellules **contenu** dans le disque, soit une seule cellule pour une empreinte de 1.5 m à résolution 1 m, alors que le disque déborde sur les huit voisines. C'est une sous-approximation, donc non conservative ;
- l'élément circulaire `di * di + dj * dj > radius * radius`, envisagé un temps pour obtenir des contours moins rectangulaires, retire encore des cellules au carré et va donc dans la mauvaise direction. Mesuré : à 3 m d'empreinte, il fait passer les faux acceptables dangereux de 0 à 2.

La convention retenue couvre le disque, et au delà de 7 m d'empreinte elle retire bien les coins du carré, ce qui satisfait aussi l'intention esthétique initiale.

**Taille minimale détectable.** Pour une zone isolée de largeur `W` cellules entourée de terrain refusé, avec `ra` le rayon d'analyse, `rf` le rayon d'empreinte :

```
W >= 2 * (ra + marge + rf) + 1
```

| Configuration | W minimale, élément sous-approximant | W minimale, élément couvrant |
|---|---|---|
| empreinte 5 m, marge 1 m, résolution 1 m | 9.0 m | n/a, à recalculer |
| empreinte 3 m, résolution 1 m | 5.0 m | n/a, à recalculer |
| empreinte 1.5 m, résolution 1 m, **configuration courante** | 3.0 m | **5.0 m** |
| empreinte 1.5 m, résolution 0.5 m | 3.5 m | **3.5 m** |

**Point à expliquer dans le mémoire, reformulé le 17 octobre 2026.** La version initiale de cet argument affirmait qu'affiner la résolution n'abaisse pas la taille minimale détectable. Avec l'élément couvrant, c'est faux : passer de 1 m à 0.5 m fait descendre la taille minimale de 5.0 à 3.5 m. La formulation correcte est en deux temps :

1. **Affiner la résolution aide, mais seulement jusqu'à une limite.** Le gain vient de ce que l'empreinte et la marge s'expriment plus finement, donc le terme `rf` diminue en mètres.
2. **La limite est la fenêtre d'analyse, pas la grille.** Cette fenêtre doit garder une taille physique d'environ 1 m pour contenir le bruit : avec 0.24 m d'erreur de carte, une demi largeur de 0.5 m donnerait un bruit de pente très supérieur au seuil de 6°, ce qui détruirait la séparation avec la pente de 10°. Le terme `ra` est donc plancher en mètres, quelle que soit la résolution.

Conséquence inchangée et toujours centrale : **le vrai levier pour détecter des zones plus petites est la réduction de l'erreur de carte**, donc voler plus bas et repasser plus souvent, donc dépenser plus d'énergie. **Cet arbitrage est exactement ce que C2 formalise**, et c'est un des meilleurs arguments du mémoire. La reformulation le renforce plutôt que l'affaiblir, puisqu'elle sépare proprement ce qui relève de la représentation, la résolution, et ce qui relève de la mesure, l'erreur de carte.

**Critère d'acceptation, révisé le 17 octobre 2026.** Deux critères de nature différente, à ne plus mettre sur le même plan.

- **Barrière, non négociable.** Zéro faux acceptable dangereux, c'est à dire zéro cellule déclarée sûre dont la vérité terrain dépasse un seuil de sûreté. **Et sur au moins trois vols indépendants**, pas sur un seul : un zéro obtenu sur une carte unique n'est pas un résultat, c'est une observation.
- **Grandeur rapportée, avec un plancher justifié.** Le recall n'est pas une barrière mais une mesure d'utilité. Le plancher de 95 % initialement inscrit ici était un chiffre rond sans justification. Le plancher correct est celui en dessous duquel le système ne trouve plus de zone assez tôt pour que la mission aboutisse, et il se déduit des résultats de 5.3, pas d'une convention. En attendant cette mesure, le recall est rapporté tel quel.

**Il n'y a pas d'arbitrage ici** : un faux acceptable dangereux est un crash, un faux rejet est une zone manquée. Les deux ne se compensent pas, et la barrière passe devant.

État au 17 octobre 2026, carte v6, empreinte 1.5 m sans marge : recall 94.2 %, faux acceptables bénins 46, faux acceptables dangereux 0. Avant correction de l'élément structurant : 96.2 %, 22 et **25**. L'affirmation antérieure de zéro faux acceptable dangereux avant cette correction était erronée.

**Mise en garde sur l'attribution.** Le zéro n'est atteint que sur v6. Sur les cartes antérieures l'élément couvrant réduit les dangereux sans les annuler, par exemple de 154 à 116 sur v1. Le résultat de sécurité est donc une propriété **conjointe de la carte et du détecteur**, pas une propriété du détecteur seul, et le mémoire doit le dire ainsi.

### S9. Registre SLZ dynamique

**Rôle.** Maintenir la liste des zones sûres confirmées. C'est la structure autour de laquelle tourne toute la contribution, et elle n'existe pas encore.

**Enregistrement par zone.**

| Champ | Type | Rôle |
|---|---|---|
| `id` | entier stable | une zone garde son identité entre deux mises à jour |
| `centroid_ned` | (nord, est) | point de référence pour les calculs d'énergie |
| `touchdown_ned` | (nord, est) | point visé, pas nécessairement le centroïde si la zone est concave |
| `area_m2` | réel | aire des cellules acceptées |
| `inscribed_radius_m` | réel | plus grand cercle inscrit, le vrai critère de posabilité |
| `n_observations` | entier | minimum sur les cellules de la zone |
| `max_variance` | réel | maximum sur les cellules, pour le gate de confiance |
| `n_passes` | entier | nombre de passages distincts ayant observé la zone, pour I4 |
| `state` | énumération | `CANDIDATE`, `CONFIRMED`, `RETIRED` |
| `first_seen`, `last_updated` | horodatages | |

**Cycle de vie.**

- Une zone détectée pour la première fois entre comme `CANDIDATE`. Elle n'est **pas** utilisable par C1.
- Elle passe à `CONFIRMED` quand elle satisfait simultanément : `inscribed_radius_m >= footprint_requis`, `max_variance <= var_confirm`, `n_passes >= 2`. L'exigence de deux passages distincts est l'invariant I4 : une observation unique sous un seul angle ne confirme rien.
- Elle est **rétrogradée** à `CANDIDATE` ou passée à `RETIRED` si une mise à jour de la carte invalide ses cellules.

**Appariement entre cycles.** Les zones sont redétectées à chaque cycle depuis une carte qui change. Il faut donc un appariement entre les zones du cycle précédent et celles du cycle courant, par recouvrement de cellules, avec un seuil de recouvrement. Sans cet appariement, les identifiants changent à chaque cycle et le compteur `n_passes` ne veut rien dire.

**S9.5. Cas critique, le rétrécissement du registre.** Le drone se dirige vers une frontière sur la base d'une garantie reposant sur la zone Z. En route, une nouvelle observation invalide Z, qui passe à `RETIRED`. Le système doit :

1. détecter l'événement **au moment de la mise à jour du registre**, pas au prochain cycle de sélection de frontière,
2. réévaluer immédiatement l'invariant I1 contre le registre amputé,
3. si l'invariant tient encore avec une autre zone, continuer et journaliser,
4. s'il ne tient plus, interrompre la navigation en cours et engager un retour vers la zone atteignable la plus proche,
5. s'il n'existe plus aucune zone atteignable, journaliser une **violation d'invariant** et déclencher la politique d'urgence.

**Ce cas est le cœur de la contribution.** C'est ce qu'aucun travail antérieur sur l'exploration sous contrainte énergétique ne traite, puisque tous supposent une base de retour statique et prédéfinie. Il doit être **provoqué délibérément** dans au moins une expérience du mémoire, et la réaction du système doit être tracée et présentée en figure.

**Critère d'acceptation.** Un scénario scripté où une zone confirmée est invalidée pendant un transit produit la séquence attendue, journalisée, sans violation de I1.

### S10. Modèle d'énergie

**Rôle.** Prédire l'énergie d'un déplacement avec une erreur assez faible pour fonder une garantie. C'est le composant **le plus risqué** du projet, et c'est pourquoi il passe avant la détection de frontières dans l'ordre de livraison.

**Forme retenue, révisée le 17 octobre 2026.** La forme `E = k * d`, inscrite au stade de la proposition, est **réfutée par la mesure**. Calée sur le survey elle donne k = 48 J/m et prédit 77 J pour un vol stationnaire de 60 s qui en consomme environ 21 kJ. Le temps de vol explique 87 % de l'énergie du survey. C'est attendu physiquement pour un multirotor, qui dépense l'essentiel de son énergie à ne pas tomber, et cela devait être mesuré plutôt que supposé.

Forme retenue, à deux termes :

```
E = P_hover * t  +  k_move * d
```

**Point important pour C1 et C2 : cette forme ne complique pas l'invariant.** À vitesse de croisière constante, `t = d / v`, donc

```
E_transit = (P_hover / v_cruise + k_move) * d  =  k_eff * d
```

La linéarité en distance est préservée **à l'intérieur du régime de transit**. Ce qui est réfuté, c'est l'usage d'un seul `k` **à travers les régimes**, transit et stationnaire confondus. Conséquence sur la spécification de C1 : chaque terme doit comptabiliser séparément le temps passé sans se déplacer.

```
E_go     = k_eff * d(p_courante -> f)  +  P_hover * t_overhead
E_return = k_eff * d(f -> slz*)
E_land   = P_hover * t_descente  +  terme de poser
```

`t_overhead` couvre la stabilisation au point d'arrivée et le temps de calcul en vol stationnaire. Il est petit mais non nul, et l'omettre rend le modèle optimiste, ce qui est le mauvais sens pour une garantie de sécurité.

**Si la calibration montre en plus une dépendance nette à la vitesse ou à la montée, le modèle est étendu, mais seulement sur la base de cette mesure.**

Trois termes distincts :

| Terme | Définition |
|---|---|
| `E_go` | énergie du point courant à la frontière candidate, via le chemin planifié |
| `E_return` | énergie de la frontière à la zone sûre minimisant ce coût |
| `E_land` | réserve calibrée séparément pour la descente contrôlée et le poser |
| `R` | réserve de sécurité fixe, en pourcentage de la capacité |

**Source du signal d'énergie, tranchée le 17 octobre 2026.** La télémétrie de batterie de PX4 SITL **ne peut pas servir** : `current_a` vaut -1 en permanence, `discharged_mah` reste à 0, et `remaining` décroît linéairement avec le **temps armé** avant de se bloquer à 0.9501. Ce blocage n'est pas un mystère, il vient de `SIM_BAT_MIN_PCT 95` posé dans l'airframe, et la pente vient de `SIM_BAT_DRAIN 3000`. Même sans ces deux paramètres, le signal resterait fonction du temps armé et non de la charge : régresser cette télémétrie sur la distance ne mesurerait que la durée du vol, et la procédure serait circulaire.

Le signal retenu est la **puissance mécanique rotor**, calculée depuis les consignes que PX4 envoie à Gazebo. L'instrument est validé par le vol stationnaire, où la prédiction analytique par le poids et la poussée tombe à 0.2 % de la mesure :

| Stationnaire | Prédit | Mesuré |
|---|---|---|
| vitesse rotor | 806.0 rad/s | 805.5 rad/s |
| puissance | 286.5 W | 285.9 W |

**Limite à déclarer dans le mémoire.** Ce signal est l'énergie **mécanique** du modèle de rotor du simulateur. Il n'inclut ni le rendement moteur et ESC, ni l'affaissement de tension, ni les effets aérodynamiques du vol d'avancement non modélisés par le greffon. Il est donc cohérent et prédictif **à l'intérieur de la simulation**, ce qui suffit à l'étude, mais il ne transfère pas tel quel à un drone réel. Pour que les durées de mission restent plausibles, le budget est converti en énergie électrique par un rendement constant déclaré, de l'ordre de 0.70 à 0.75, posé comme hypothèse unique et soumis à une analyse de sensibilité.

**Procédure de calibration, révisée.**

1. Voler des trajectoires de longueur connue et croissante, par exemple des carrés de 10, 20, 40 et 60 m à altitude constante, trois répétitions chacun, **à vitesse de croisière fixée et journalisée**.
2. Enregistrer la puissance rotor instantanée et la position, et intégrer l'énergie par segment.
3. Régresser l'énergie sur **deux régresseurs**, le temps et la distance, et estimer `P_hover` et `k_move` avec leurs intervalles de confiance. Vérifier que `P_hover` retrouve la valeur du stationnaire pur, ce qui est le contrôle de cohérence de la régression.
4. Calibrer `E_land` séparément : dix descentes depuis l'altitude de croisière jusqu'au poser, énergie moyenne et écart type.
5. Valider sur des trajectoires **non utilisées pour la calibration**, dont au moins une comportant une phase stationnaire prolongée, puisque c'est le régime qui a réfuté le modèle initial.

**Dimensionnement du budget.** Le budget énergétique de l'expérience est un budget **logique**, géré par le gestionnaire d'exploration, et non la batterie simulée de PX4. Il doit être dimensionné pour **mordre** : si une mission de couverture complète tient confortablement dans le budget, la configuration de référence B1 n'échouera jamais et l'expérience ne prouvera rien. Viser un budget de l'ordre de la moitié à deux tiers de ce qu'exige une couverture complète du monde considéré.

**Validation, faisable dès maintenant sans frontières.** Prédire la consommation du survey existant, comparer à la mesure PX4, reporter l'erreur relative. Une trajectoire connue est un bien meilleur banc de test qu'une exploration autonome où tout varie à la fois.

**Critère d'acceptation.** Erreur relative de prédiction inférieure à 10 % sur des trajectoires non utilisées pour la calibration. Si cette barre n'est pas tenue, le modèle doit être enrichi ou la réserve `R` augmentée en conséquence, et la décision documentée.

**Calibration du 19 octobre 2026, porte C passée** (`experiments/2026-10-19_energy_calibration`). Seize vols sur PX4 et Gazebo neufs : carrés de 10, 20, 40 et 60 m répétés trois fois à 5 m/s, avec un stationnaire de 10, 20 ou 30 s avant chaque tour pour décorréler temps et distance, plus quatre vols de validation jamais ajustés. La vitesse est fixée par construction : chaque segment suit un profil trapézoïdal de repos à repos, 3 m/s² puis 5 m/s, envoyé en position, vitesse et accélération. Une consigne avançant à vitesse constante a été essayée puis rejetée : 56.3 m parcourus pour un carré de 40 m, vitesse entre 4.3 et 7.9 m/s.

**Forme adoptée, révisée le 19 octobre 2026 : un terme par segment.**

```
E = P_hover * t  +  k_move * d  +  c_leg * n_segments
```

La forme à deux termes passe la barre des 10 % d'un facteur dix, mais son erreur est structurée et **optimiste** : -4.2 % sur les transits de 10 m, -1.00 % depuis le plan sur un zigzag de 8 segments. Chaque segment paie les mêmes deux rampes. Avec ce terme, les erreurs de transit tombent dans ±0.16 % et le zigzag, hors ajustement, à -0.14 % depuis le plan. Le zigzag, avec 8 segments contre 4 pour chaque carré, montre que le coût est bien par segment et non par transit. `n_segments` est connu dès le plan, donc C1 peut l'utiliser.

| Paramètre | Valeur, IC 95 % |
|---|---|
| `P_hover` | 285.64 ± 0.12 W, contre 285.9 W en stationnaire pur |
| `k_move` | -0.400 ± 0.042 J/m |
| `c_leg` | 65.2 ± 1.3 J par segment |
| `E_land` | 3199 J, écart type 15 J, 12 descentes depuis 8 m |
| temps d'un segment | `L / v + v / a`, exact depuis le plan, plus 0.06 s de stabilisation à l'arrivée |

Trois points à dire dans le mémoire. Les intervalles mesurent le défaut de forme du modèle et non un bruit : la simulation se répète à 0.02 % près, trois répétitions ne sont pas trois échantillons indépendants. Dans ce simulateur l'énergie est presque entièrement du temps, `k_move` négatif signifiant que la croisière à 5 m/s consomme 2 W de moins que le stationnaire, origine non établie. Le modèle ne vaut que pour le profil calibré : l'exécuteur de trajectoire doit voler ce profil, avec arrêt à chaque sommet, ou le modèle doit être recalibré. Implémentation : `uav_core/energy_model.py`.

**Note sur `SIM_BAT_DRAIN` et `SIM_BAT_MIN_PCT`.** Ces deux paramètres de l'airframe allongent la batterie simulée bien au delà de toute mission. Ils ont été ajoutés en croyant corriger un failsafe, ce qui était un faux diagnostic, la vraie cause était le timestamp. Ils sont conservés, et ils expliquent entièrement la forme du signal `remaining` décrite plus haut, pente et blocage à 95 %. Les garder est cohérent avec un budget géré dans le code ; les retirer ne rendrait pas la télémétrie utilisable pour autant, puisqu'elle resterait fonction du seul temps armé.

### S11. Détection de frontières

**Rôle.** Produire le catalogue des endroits où aller pour apprendre quelque chose. **Sur la carte, jamais sur le nuage brut** : c'est le point explicite du retour du superviseur.

**Définition.** Une cellule frontière est une cellule `OBSERVED` ayant au moins un voisin `UNKNOWN` en 8-connexité.

**Traitement.**

1. Extraction du masque de frontières depuis la couche d'état d'observation.
2. Regroupement par remplissage par diffusion en 8-connexité.
3. Pour chaque groupe : centroïde, taille en cellules, point d'accès accessible le plus proche du centroïde.
4. Filtres de rejet :
   - groupes plus petits que `min_cluster_cells`,
   - groupes dont le centroïde est à moins de `boundary_margin_m` du bord de la grille, artefacts de bord,
   - groupes dont le centroïde tombe sur une cellule dont la densité d'observation est anormalement basse, ce qui en environnement côtier correspond à l'eau, qui absorbe l'infrarouge du LiDAR,
   - groupes en liste noire, voir ci dessous.
5. **Gain d'information** `G(f)` par cellule candidate : nombre de cellules `UNKNOWN` dans un rayon correspondant à la portée utile du capteur depuis le point d'accès, estimé par comptage et non par lancer de rayons complet, sauf si la mesure montre que le comptage surestime fortement.

**Liste noire escaladante.** Une frontière visitée sans que son état change doit être écartée temporairement, puis définitivement : 30 secondes au premier échec, 300 secondes après trois échecs non résolus. Sans ce mécanisme, le drone cycle indéfiniment entre deux ou trois points. Dans l'ancienne architecture, ce comportement consommait environ 65 % du temps de vol avant correction.

**Critère d'acceptation.** Sur le monde de validation, le détecteur produit un catalogue non vide tant qu'il reste des cellules inconnues atteignables, et vide quand la couverture est complète. La terminaison est une propriété à démontrer, pas à espérer.

### S12. Planification de chemin

**Rôle.** Fournir un chemin sans collision et sa longueur, qui est l'entrée du modèle d'énergie.

**Spécification.** A* 2D sur la grille, à l'altitude de croisière, sur une couche d'obstacles dérivée de la carte : une cellule est bloquée si son élévation dépasse `altitude_croisiere - marge_verticale`, ou si elle est inconnue et que la politique est conservatrice.

Exigences issues des échecs de l'ancienne architecture :

- **Inflation des obstacles** par le rayon du drone, avec récupération si la cellule de départ se retrouve inflatée.
- **Horizon glissant** dimensionné sur la distance de freinage à la vitesse courante, et non sur une constante.
- Lors d'un replanning à horizon partiel, **insérer un point de passage intermédiaire sur la droite vers l'objectif réel**. Sans cela, la sélection du nœud terminal par distance depuis le départ plutôt que par direction vers le but produit une dérive systématique hors de l'objectif à chaque replan successif.
- **Confiance d'obstacle par ratio** `hits / observations`, pas par compteur absolu : un compteur absolu ne décroît jamais et fige une erreur transitoire.
- **Temporisation de replanification** suffisante, de l'ordre de 10 s, sinon le drone replanifie en boucle sans avancer.
- Un **chien de garde** à 5 Hz vérifie que les cellules du chemin courant restent libres et déclenche une replanification immédiate sinon.
- Le planificateur est isolé derrière une interface d'action `PlanPath`, pour qu'en changer n'affecte qu'un seul fichier.

**Critère d'acceptation.** Le chemin retourné est libre d'obstacles connus, sa longueur est cohérente à moins de 5 % avec la distance réellement parcourue par l'exécuteur, puisque c'est elle qui alimente le modèle d'énergie.

### S13. C1, invariant de faisabilité énergétique

**Rôle.** Filtrer les frontières candidates. C'est un **filtre booléen**, appliqué avant tout classement.

**Algorithme.**

```
pour chaque frontiere candidate f :
    E_go = energie(chemin(p_courante, f))
    si registre vide :
        slz* = point de decollage, marque comme provisoire
    sinon :
        slz* = argmin sur les zones CONFIRMED de energie(chemin(f, z))
    E_return = energie(chemin(f, slz*))
    admissible(f) = (E_go + E_return + E_land <= B_current - R)
```

**Points de spécification souvent négligés, à traiter explicitement.**

- Le chemin `f -> slz*` doit être calculé sur **la carte telle qu'elle sera connue à l'arrivée en f**, ce qui est inconnaissable. Approximation retenue : la carte courante. Cette approximation est **optimiste** et doit être compensée par la réserve `R`, et dite comme telle dans le mémoire.
- La réévaluation de l'invariant doit être **périodique** et pas seulement à la sélection : à chaque mise à jour du registre, à chaque mise à jour significative de `B_current`, et au minimum à 1 Hz.
- Si l'invariant devient faux pendant un transit, cela ne doit **jamais** être silencieux : journalisation explicite avec la cause, l'énergie restante, et la zone visée.
- Le cas où **aucune** frontière n'est admissible est un état légitime de la mission, pas une erreur : il signifie que l'exploration est terminée pour raison énergétique, et doit conduire au poser.
- **Le `t_overhead` de S10 est une latence de décision, à mesurer et non à supposer**, ajouté le 19 octobre 2026. C1 doit couvrir tout le temps hors plan entre la lecture de `B_current` et la commande suivante : le calcul avant le départ du chemin, la stabilisation, l'attente et le calcul à la frontière. Une constante de 2 s a suffi tant que les décisions duraient moins d'une seconde ; sur une zone de 190 m, la décision a crû avec la carte jusqu'à 21.5 s, 301 s de stationnaire n'ont jamais été comptées et I1 a été violé 7 fois avec C1 actif. Depuis a4ac769, la machine à états mesure ce temps à chaque visite et facture le maximum des cinq dernières plus la plus forte hausse récente (`experiments/2026-10-19_decision_latency`). La première visite reste facturée à la constante, faute de mesure.

**Critère d'acceptation.** Sur un ensemble de missions, **zéro violation de I1** avec C1 actif. Et pour que la mesure veuille dire quelque chose, les mêmes missions sans C1 doivent produire des violations : si la configuration de référence n'échoue jamais, l'expérience ne prouve rien et le budget énergétique doit être resserré jusqu'à ce que le problème existe.

### S14. C2, utilité à marge énergétique

**Rôle.** Classer les frontières admissibles.

```
G(f) = cellules_inconnues_gagnees(f) / max sur les candidates
M(f) = clip((B_current - R - E_total(f)) / (B_current - R), 0, 1)
U(f) = alpha * G(f) + (1 - alpha) * M(f)
```

**Exigences.**

- La normalisation de `G` se fait **sur l'ensemble des candidates du cycle courant**, pas sur une constante globale, sinon `alpha` n'a pas le même sens d'un cycle à l'autre.
- `alpha` est un paramètre ROS modifiable à chaud, pour permettre un balayage sans recompilation.
- Les valeurs de `U`, `G`, `M` et `E_total` de **toutes** les candidates de chaque cycle sont journalisées, pas seulement celles de la gagnante. C'est ce qui permet de produire les figures d'analyse du comportement de sélection.

**Critère d'acceptation.** Le balayage de `alpha` sur cinq valeurs produit une variation **monotone et interprétable** d'au moins une métrique. Si aucune métrique ne bouge, C2 n'a pas d'effet mesurable et c'est un résultat à publier comme tel, avec son explication.

### S15. Gestionnaire d'exploration

**Rôle.** Détenir l'autorité de décision. Un seul composant décide.

**S15.1. Machine à états.**

| État | Rôle | Transition de sortie |
|---|---|---|
| `INIT` | attend pose valide, carte prête, calibration chargée | tout prêt |
| `TAKEOFF` | monte à l'altitude de croisière | altitude atteinte |
| `BOOTSTRAP` | premier balayage local court pour amorcer la carte | carte locale exploitable |
| `SELECTING` | détecte frontières, applique C1, classe par C2 | une candidate retenue, ou aucune |
| `NAVIGATING` | suit le chemin, réévalue l'invariant | arrivée, préemption, ou invariant menacé |
| `REPLANNING` | chemin invalidé par le chien de garde | nouveau chemin |
| `RETURNING` | transit vers la zone sûre retenue | arrivée au dessus de la zone |
| `COMMIT_LANDING` | descente engagée, plus de réévaluation d'exploration | poser |
| `LANDED` | désarmement, sauvegarde finale | fin |
| `EMERGENCY` | aucune zone atteignable, violation journalisée | poser au mieux |

**S15.2. Point d'engagement.** Une fois la descente commencée, le drone ne doit **pas** changer d'avis parce qu'une meilleure zone apparaît : l'oscillation en fin de mission est le pire comportement possible du point de vue énergétique. L'engagement est irréversible sauf détection d'obstacle sur le site visé.

**S15.3. Amorçage, le cas t = 0.** Avant la première zone confirmée, le registre est vide et C1 n'a rien à filtrer. Deux politiques possibles :

- **(a)** Le point de décollage est inscrit au registre comme zone sûre **par construction** : le drone en a décollé, donc il peut s'y poser. Il est marqué `provisional = True` pour que les mesures puissent distinguer les missions terminées sur une zone découverte de celles terminées au point de départ.
- **(b)** Avant la première zone confirmée, l'exploration est contrainte par une simple réserve de retour au point de départ, et C1 ne s'active qu'ensuite.

**Politique recommandée : (a).** Elle est plus simple, elle rend l'invariant I1 vrai dès t = 0 sans cas particulier, et elle est défendable en une phrase devant un jury. Le choix doit toutefois être **explicité dans le mémoire** avec cette justification, parce que c'est exactement le point que la clause conditionnelle de la question de recherche cherche à couvrir.

**S15.4. Conditions de fin de mission.** Trois, toutes légitimes, toutes journalisées distinctement :

1. **Couverture complète** : plus aucune frontière valide.
2. **Épuisement du budget** : plus aucune frontière admissible sous C1.
3. **Arrêt commandé** : terminaison externe.

Dans les trois cas, le drone se pose sur une zone du registre, jamais où il se trouve.

### S16. Atterrissage et précision de poser

**Rôle.** Terminer la mission, et fournir le chiffre qui dimensionne tout le détecteur.

**Problème ouvert P4, à traiter.** L'empreinte de 1.5 m repose sur une estimation, pas sur une mesure : envergure du x500 avec hélices d'environ 0.70 m, plus une dispersion d'atterrissage estimée à 0.40 m de chaque côté avec `eph` autour de 0.15 m. **C'est le paramètre le plus structurant du détecteur et il n'est pas mesuré.**

**Expérience à mener, simple et rapide.**

1. Dix à vingt atterrissages depuis l'offboard sur la même consigne, sur terrain plat, même altitude de départ.
2. Enregistrer la position de poser et la consigne.
3. Calculer l'erreur radiale moyenne, l'écart type, le 95e percentile.
4. Fixer `footprint_m = 2 * (rayon_airframe + erreur_radiale_p95)`, arrondi vers le haut.
5. Relancer la validation SLZ avec cette valeur et reporter l'effet sur le recall et sur la taille minimale détectable.

**Critère d'acceptation.** Le rayon d'empreinte utilisé dans le mémoire est issu d'une mesure, avec son histogramme en figure.

### S17. Journalisation et rejouabilité

**Rôle.** Rendre chaque résultat du mémoire reproductible et chaque comportement explicable après coup. Ce composant n'est pas accessoire : sans lui, les figures ne sont pas défendables.

**À enregistrer pour chaque mission, dans un seul répertoire horodaté :**

- la configuration complète, tous les paramètres, la graine, la version de la carte, le hash git,
- la carte d'élévation finale et des instantanés périodiques,
- la trajectoire réelle, horodatée,
- l'état du registre SLZ à chaque mise à jour, avec les transitions d'état,
- pour chaque cycle de sélection, **toutes** les candidates avec `G`, `M`, `E_go`, `E_return`, `U`, et l'admissibilité,
- l'énergie estimée et l'énergie mesurée PX4, en parallèle,
- chaque évaluation de l'invariant I1, avec sa marge résiduelle,
- les événements : rétrogradation de zone, mise en liste noire, replanification, engagement de descente.

**Critère d'acceptation.** Toute figure du mémoire peut être régénérée depuis un répertoire de mission par un script, sans relancer la simulation.

---

## 5. Protocole d'évaluation

C'est la partie qui transforme un système en travail de recherche. Elle doit être conçue **avant** que les résultats n'arrivent, pour ne pas choisir les métriques après coup.

### 5.1 Mondes de test

Trois, avec leur vérité terrain générée analytiquement comme en S2.

| Monde | Caractère | Ce qu'il teste |
|---|---|---|
| **W1, validation** | 60 x 60 m, zones connues et isolées, pente 3°, pente 10°, zone rugueuse, cube | justesse de la carte, de l'analyse et du détecteur, pas l'exploration |
| **W2, rare** | grand, majoritairement non posable, deux ou trois zones sûres éloignées | met C1 sous tension : les zones sûres sont loin, la marge se consomme vite |
| **W3, trompeur** | contient une zone qui paraît sûre vue de loin et de haut, et qui ne l'est pas vue de près | provoque le rétrécissement du registre, cas critique S9.5 |

W3 est celui qui produit la figure la plus intéressante du mémoire. Il doit être construit exprès.

### 5.2 Configurations comparées

| Code | Configuration |
|---|---|
| **B0** | exploration par frontière la plus proche, aucune contrainte énergétique |
| **B1** | exploration par gain d'information pur, retour déclenché à un seuil de batterie fixe, la référence classique de la littérature |
| **B2** | C1 actif, sélection par gain d'information pur parmi les admissibles |
| **B3** | C1 + C2, `alpha` balayé sur {0.0, 0.25, 0.5, 0.75, 1.0} |

B1 est la vraie référence à battre, pas B0. Un seuil de batterie fixe avec retour à la base est ce que fait l'état de l'art, et c'est contre cela que la contribution doit se mesurer.

### 5.3 Métriques

Chacune doit avoir une définition opératoire, pas une intention.

**Sûreté, les plus importantes.**

| Métrique | Définition |
|---|---|
| `violations_I1` | nombre d'instants où le registre est non vide et aucune zone n'est atteignable. **Doit valoir 0 avec C1.** |
| `marge_min` | plus petite marge énergétique résiduelle atteinte pendant la mission |
| `poser_sur_zone_sure` | booléen, le poser final a eu lieu sur une zone dont la vérité terrain est sûre |
| `faux_acceptables_dangereux` | cellules déclarées sûres dont la vérité dépasse un seuil de sûreté. **Doit valoir 0.** |

**Qualité de perception.**

| Métrique | Définition |
|---|---|
| `rms_elevation` | erreur RMS d'élévation par zone, contre vérité |
| `couverture` | fraction de cellules passées à `OBSERVED` |
| `recall_slz`, `precision_slz` | par cellule, contre la vérité terrain |
| `temps_premiere_slz` | temps écoulé jusqu'à la première zone `CONFIRMED` |

**Efficacité.**

| Métrique | Définition |
|---|---|
| `couverture_par_joule` | cellules observées par unité d'énergie dépensée |
| `energie_au_poser` | énergie restante à l'instant du poser, mesure du conservatisme |
| `distance_moyenne_a_slz` | distance à la zone sûre la plus proche, moyennée sur le temps de mission |
| `taux_achevement` | fraction des missions terminées par un poser sur zone sûre |

**Qualité du modèle d'énergie.**

| Métrique | Définition |
|---|---|
| `erreur_prediction` | écart relatif entre énergie prédite et énergie mesurée PX4, par segment |
| `biais_prediction` | signe moyen de cet écart ; un biais optimiste est plus grave qu'un biais pessimiste |

### 5.4 Plan expérimental

- **10 runs par configuration et par monde.** Les mêmes graines sont utilisées pour toutes les configurations, de sorte que les comparaisons soient appariées.
- 3 mondes x 4 configurations, plus 5 valeurs de `alpha` pour B3, soit 3 x (3 + 5) x 10 = **240 missions**. À environ 3 minutes par mission plus le traitement, c'est faisable en quelques jours de machine, à condition que le lancement et le dépouillement soient **scriptés de bout en bout**. Prévoir ce script, c'est l'investissement qui rend le chapitre résultats possible.
- Résultats rapportés en **médiane et écart interquartile**, pas en moyenne : avec 10 runs et des distributions non normales, la moyenne ment.
- Comparaisons appariées par test de Wilcoxon, avec la taille d'effet, pas seulement une valeur p.
- Toute mission qui échoue pour une raison technique, perte de lien offboard, plantage du simulateur, est **relancée et signalée**, jamais écartée silencieusement. Le nombre de relances est rapporté.

### 5.5 Ce qui constitue un succès

Le projet réussit si, à la fin, on peut écrire ces phrases en les ayant mesurées :

1. La carte permet de classer le terrain avec un recall supérieur à 95 % et **zéro** faux acceptable dangereux sur les trois mondes.
2. Le modèle d'énergie prédit la consommation à mieux que 10 % sur des trajectoires non calibrées.
3. Avec C1, le système n'entre **jamais** dans un état où une zone sûre connue est devenue inatteignable, alors que la référence B1 y entre dans une fraction mesurable des missions.
4. L'effet de `alpha` sur l'arbitrage couverture contre marge est mesuré et monotone, et un réglage recommandé est justifié.
5. Le cas du rétrécissement du registre est provoqué, tracé, et le système y réagit correctement.

**Le point 3 est la thèse.** Les autres sont ce qui la rend crédible.

---

## 6. Ce que le mémoire doit contenir

### 6.1 Structure

1. **Introduction.** Vision, par exemple réponse à catastrophe et surveillance côtière, puis défi, puis question de recherche, puis méthode en un paragraphe, puis annonce des résultats attendus.
2. **Revue de littérature.** Déjà écrite en Capstone A. Fil conducteur : la détection géométrique de sites d'atterrissage est le thème principal, l'énergie est une contrainte de faisabilité physique qui émerge dans les limites de chaque section, pas une section isolée. La section finale argumente que tous les travaux sous contrainte énergétique supposent un site de retour **statique et prédéfini**, et nomme les quatre propriétés absentes de la littérature : les états sûrs ne sont pas prédéfinis, ils sont découverts en ligne, ils sont géométriquement incertains jusqu'à confirmation, et ils doivent rester atteignables sous un budget énergétique décroissant, y compris avant qu'un seul candidat n'ait été trouvé.
3. **Méthode.** Architecture, repères, cartographie, analyse, détection, registre, énergie, C1, C2, politique d'exploration. Chaque choix de conception avec sa justification et, quand elle existe, la mesure qui l'a dictée.
4. **Dispositif expérimental.** Mondes, vérité terrain, configurations, métriques, plan expérimental.
5. **Résultats.** Dans l'ordre : perception, énergie, invariant, politique.
6. **Discussion.** Y compris les résultats négatifs et les limites.
7. **Conclusion et travaux futurs.**

### 6.2 Figures et tableaux obligatoires

| N° | Contenu |
|---|---|
| F1 | architecture du système, nodes et flux, avec les contributions en trait discontinu et l'état de l'art en trait plein |
| F2 | les quatre repères et les conversions entre eux |
| F3 | le monde de validation, rendu Gazebo et carte de vérité côte à côte |
| F4 | carte d'élévation contre vérité, carte de différence, par zone |
| F5 | histogramme du plancher de bruit de pente avec p95, et p01 de la pente de 10°, **la figure qui justifie le seuil de 6°** |
| F6 | séparation pente contre rugosité, nuage de points par zone |
| F7 | zones détectées superposées à la vérité, faux acceptables bénins et dangereux en couleurs distinctes |
| F8 | régression de calibration énergétique, avec intervalle de confiance et résidus |
| F9 | chronogramme d'une mission : énergie restante, énergie requise pour rejoindre la zone la plus proche, marge, avec les événements du registre annotés |
| F10 | la même chose pour la référence B1, montrant le franchissement de la limite |
| F11 | balayage de `alpha` : couverture et marge minimale en fonction de `alpha` |
| F12 | trajectoires superposées, B1 contre B3, sur W2 |
| F13 | le cas du rétrécissement du registre sur W3, carte avant et après invalidation, avec la trajectoire de réaction |
| F14 | histogramme de précision d'atterrissage, avec le rayon d'empreinte retenu |
| T1 | tableau de tous les paramètres du système avec leur valeur, leur origine (mesuré, choisi, hérité) et la section qui les justifie |
| T2 | tableau des résultats principaux, toutes configurations, toutes métriques, médiane et IQR |

Le tableau T1 est celui que les jurys apprécient le plus et que les étudiants oublient le plus souvent.

### 6.3 Résultats négatifs à inclure

Trois, déjà obtenus, chacun avec son explication. Ils renforcent le mémoire au lieu de l'affaiblir.

1. **Variance croissant avec la distance d'observation.** Le modèle `sigma = 0.02 + 0.009 * d` a dégradé l'erreur de carte de 0.1209 à 0.1433 m. Explication : l'erreur dominante n'est pas un bruit indépendant mais une erreur d'assiette **corrélée entre les points d'un même scan**, alors que le filtre de Kalman suppose des observations indépendantes. En donnant tout le poids à l'observation la plus proche, on ramène le nombre d'observations réellement indépendantes à une. Conséquence utile : **c'est la diversité angulaire entre passages qui compte, pas la pondération par point.**
2. **Porte de Mahalanobis aux discontinuités.** Écrite puis retirée : les faux acceptables dangereux sont passés de 26 à 138 et le recall de 92.6 % à 65.3 %. Cause de l'échec : la logique de patience exigeait cinq aberrants **consécutifs**, or avec deux populations entrelacées le compteur se remet à zéro en permanence, donc la cellule reste verrouillée à jamais sur la première surface vue. Une erreur partielle devenait définitive. Leçon : tout correctif aux discontinuités doit maintenir **deux hypothèses de surface** par cellule, et non choisir irrévocablement.
3. **Affiner la résolution de grille n'abaisse pas la taille minimale détectable**, pour les raisons développées en S8. Contre-intuitif, démontré, et directement lié à l'arbitrage que C2 formalise.

### 6.4 Limites à déclarer

- Simulation seule, pas de vol réel, pas de validation sur capteur physique.
- Pas de vent, donc le modèle d'énergie est optimiste par rapport au vol réel.
- Budget énergétique **logique** et non batterie simulée réaliste, voir la note en S10.
- Environnement statique, pas de vieillissement de la carte.
- Localisation par GPS considérée fiable ; l'incertitude de pose est prise à travers `eph` mais n'est pas propagée formellement dans la carte.
- **Problème P2 non résolu** si tel est encore le cas en décembre : une seule gaussienne par cellule ne peut pas représenter deux surfaces, donc la carte rétrécit les structures surélevées d'environ 1.5 m à leurs bords. Mesuré, expliqué, non corrigé. À déclarer, pas à cacher.
- Monocapteur, un seul drone.

---

## 7. Plan de livraison et portes

Lettres fixées, utilisées pour référence dans toutes les conversations.

| | Étape | Porte de sortie, mesurable | État au 19 octobre 2026 |
|---|---|---|---|
| A | analyse de terrain, pente, rugosité, marche | pente et rugosité séparées contre vérité | **fait** |
| B | classification et registre SLZ | registre avec cycle de vie et appariement, scénario d'invalidation passant | **registre fait** (`slz_registry.py`), scénario d'invalidation passant en test scripté ; scénario en vol sur W3 à faire. Constat : I4 à deux passages ne confirme presque aucune grande zone en exploration par frontières |
| C | modèle d'énergie | erreur de prédiction inférieure à 10 % sur trajectoires non calibrées | **fait le 19 octobre**, au plus 1.0 % depuis le plan à deux termes, 0.26 % avec le terme par segment |
| D | détection de frontières sur la carte | catalogue non vide tant qu'il reste de l'inconnu, vide à couverture complète | **fait** (`frontier_detection.py`), vide à couverture complète mesuré en mission |
| E | C1, invariant | zéro violation de I1 sur 10 missions, et violations présentes sans C1 | C1 fait ; **démonstration sur un vol par politique à 20 kJ : 0 violation avec C1, 12 pour B1, 25 pour B0**. Les 10 missions restent à voler |
| F | C2, utilité | effet de `alpha` monotone et interprétable sur au moins une métrique | C2 fait, effet de `alpha` non mesuré |
| G | exploration autonome de bout en bout | mission complète sans intervention, du décollage au poser sur zone confirmée | **fait le 19 octobre** : couverture 100 % de W1 en 97 s, poser sur la zone de décollage à 0.7 m |
| H | évaluation contre référence | les 240 missions, scriptées, dépouillées, figures générées | à faire |

A, B et D sont de l'infrastructure. E et F sont la contribution. G rend l'ensemble démontrable. H le rend publiable.

### 7.1 Ordre C avant D, et pourquoi

Arbitrage déjà tranché, à ne pas rouvrir sans raison nouvelle. L'argument naïf voudrait D avant C, puisque C1 filtre des frontières et qu'un modèle d'énergie sans frontières ne filtre rien.

L'argument retenu : **C est le composant le plus risqué et le plus central.** La question de recherche porte sur la *reachability*, qui est une notion énergétique. Si le modèle d'énergie s'avère incalibrable en novembre, les deux contributions s'effondrent, alors qu'une détection de frontières imparfaite se simplifie sans dommage. De plus, C se valide **dès maintenant** sans frontières, en prédisant la consommation du survey existant. Une trajectoire connue est un bien meilleur banc de test qu'une exploration autonome où tout varie à la fois. Enfin, C ne dépend ni de A ni de B, donc il peut avancer en parallèle.

### 7.2 Calendrier jusqu'à la soutenance

Indicatif, à ajuster, mais la forme compte : **l'évaluation et la rédaction doivent occuper les six dernières semaines**, pas les six derniers jours.

| Période | Objectif |
|---|---|
| fin octobre | C, calibration énergétique validée. P4, mesure de précision de poser. Correctif de l'élément structurant circulaire. |
| début novembre | B, registre dynamique avec cycle de vie et appariement. D, frontières sur la carte. |
| mi novembre | E et F, C1 et C2 intégrés dans le gestionnaire. G, première mission autonome complète. |
| fin novembre | W2 et W3 construits. H, campagne expérimentale scriptée et lancée. |
| début décembre | dépouillement, figures, rédaction du chapitre résultats et de la discussion. |
| avant soutenance | répétition de la démonstration sur machine Linux native, avec plan de repli en vidéo enregistrée. |

**Prévoir la vidéo de secours.** Une démonstration en direct qui échoue devant un jury coûte plus cher que tout ce qu'elle peut rapporter.

---

## 8. Catalogue des pièges

Tous silencieux, aucun ne produit de message d'erreur. Cette liste évite de les repayer.

1. `TrajectorySetpoint` avec des zéros au lieu de `NaN` : PX4 reçoit des consignes contradictoires et refuse l'offboard.
2. Timestamp en temps Unix au lieu de microsecondes depuis le démarrage : message 56 000 ans dans le futur.
3. `timesync_status` à 1 Hz recopié tel quel : flux jugé périmé, retour automatique déclenché. Mémoriser le décalage, extrapoler.
4. Topics PX4 versionnés : `vehicle_local_position_v1`, `vehicle_status_v4`, `battery_status_v1`, `home_position_v1`. S'abonner à un topic inexistant ne produit aucune erreur.
5. `px4_msgs` doit être compilé avant que ROS puisse lire un topic PX4.
6. Le nom du monde dans le SDF doit correspondre à `PX4_GZ_WORLD`, sinon PX4 reste bloqué sur `Waiting for Gazebo world...`.
7. PX4 construit lui même le chemin du modèle et ignore `GZ_SIM_RESOURCE_PATH` : lien symbolique obligatoire.
8. Rayons LiDAR sans retour encodés à **zéro**, pas à l'infini. Un zéro passe tous les tests de finitude. Filtrer sur la norme.
9. Auto-détection du drone par son propre LiDAR : résolue par la géométrie, capteur à 15 cm et suppression des rayons vers le haut, pas par un filtre logiciel a posteriori.
10. `round(1.5)` vaut 2 en Python, arrondi au pair. Une empreinte demandée à 3 m en faisait 5.
11. Tranche numpy non contiguë : `.copy()` obligatoire avant `.view(np.float32)`.
12. QoS différents pour le LiDAR et pour PX4. Le mauvais profil ne produit **ni message ni erreur**.
13. `/tmp/elevation_map.npz` écrasé par tout lancement ultérieur du node de cartographie. Copier immédiatement après le vol.
14. Processus fantômes de la session précédente : la moitié des échecs de démarrage viennent de là. Nettoyer **systématiquement** avant chaque lancement, pas seulement après un plantage. Le processus `ruby` n'est pas une erreur, c'est le serveur Gazebo Harmonic, et c'est typiquement lui qui survit.
15. `rqt_graph` paraît vide tant qu'on n'a pas choisi `Nodes/Topics (all)`, décoché `Dead sinks` et `Leaf topics`, et rafraîchi.
16. L'agent XRCE-DDS n'est pas un node ROS : les topics `/fmu/` apparaissent sans source dans `rqt_graph`. À annoter sur toute figure présentée.
17. Comparaison carte contre vérité : vérifier le décalage de grille avec `check_alignment.py` avant de conclure qu'un résidu est réel.
18. Ne jamais lancer `colcon build` depuis `~/ros2_ws`, uniquement depuis l'intérieur d'un des deux workspaces.
19. PX4 SITL sauvegarde dans `~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson` ce qu'il apprend en vol, dont les offsets du magnétomètre, et les recharge au lancement suivant. Ils dérivent de vol en vol, de [0.053, -0.029, 0.212] à [0.081, 0.035, 0.286] G le 19 octobre ; au delà, `Preflight Fail: heading estimate invalid` et PX4 n'est jamais prêt. Toute mission dépendait aussi des vols d'avant. Supprimer ce fichier avant chaque lancement, voir A.3.
20. Le test de fumée du gestionnaire crée un vrai node, qui publie décollage, chemin et atterrissage sur `/executor/*`. Lancé sur le domaine ROS par défaut pendant une simulation, il commanderait son exécuteur. Il tourne dans le domaine 99 depuis le 19 octobre ; tout autre script hors ligne qui instancie `ExplorationManager` (`profile_cycle.py`) ne se lance que simulation arrêtée.
21. Le journal ne mesurait que le modèle du monde dans `timings_ms` : une décision de 21 s y apparaissait comme 179 ms. Un chronométrage partiel est pire qu'aucun, il rassure.

---

## 9. Non-objectifs

À refuser explicitement si la tentation revient, parce que chacun coûterait plusieurs semaines sans servir la question de recherche :

- apprentissage automatique pour la classification de terrain, la méthode géométrique est le sujet,
- SLAM ou odométrie visuelle,
- flotte de drones,
- optimisation de trajectoire continue, le A* en grille suffit au besoin,
- portage sur matériel réel,
- interface graphique de supervision, RViz suffit,
- optimisation de performance du code tant que la lenteur n'a pas été **mesurée** comme un problème avec `gz stats` et `htop`.

---

## 10. État au 17 octobre 2026

**Fait et mesuré.** Chaîne de contrôle offboard stable, modèle de drone et LiDAR, monde de validation avec vérité terrain analytique, cartographie 2.5D avec fusion de Kalman par cellule et plancher de variance, analyse de terrain validée contre vérité, détection SLZ avec seuils justifiés par la mesure, élément d'empreinte couvrant, instrument de mesure d'énergie validé sur le stationnaire, outillage de validation hors ligne complet. Référence courante : carte v6, erreur RMS 0.240 m sur plat, recall 94.2 %, zéro faux acceptable dangereux sur ce seul vol.

**Ouvert et identifié.**

| Code | Problème | Statut |
|---|---|---|
| P2 | la carte rétrécit les surfaces surélevées, une gaussienne par cellule ne peut représenter deux surfaces | mesuré, correctif par porte de Mahalanobis tenté et rejeté, à reprendre avec deux hypothèses de surface |
| P4 | précision d'atterrissage non mesurée, l'empreinte de 1.5 m est estimée | expérience spécifiée en S16, non exécutée |
| **P5** | **décalage d'une cellule vers le nord entre carte et vérité sur les rampes**, RMS 0.255 m contre 0.342 m après recalage, soit 25 % d'amélioration, donc un minimum franc et non plus un résidu | **nouveau, 17 octobre.** L'élément couvrant absorbe ce décalage, ce qui veut dire que le zéro dangereux repose en partie sur une compensation d'erreur. Cause à isoler avant de présenter le résultat |
| n/a | arcs concentriques d'erreur liés aux angles d'élévation des faisceaux, hypothèse d'un effet d'angle d'incidence non modélisé | observé, hypothèse non testée |
| n/a | `offboard_demo.py` et `square_mission.py` recopient l'horodatage brut de `timesync_status`, piège 3 | **corrigé le 19 octobre**, failsafes 177 → 0 et 17 → 0 sur monde neuf (`experiments/2026-10-19_offboard_timestamp`) |
| n/a | contradiction de cette spécification : S5 prescrit une variance par point croissant avec la distance, 6.3 la classe comme résultat négatif (0.1209 → 0.1433 m). `cloud_world` l'applique avec `sigma_angular_rad = 0.009` | relevé le 19 octobre, à trancher par une mesure sur la même carte |

**Note de méthode sur P5.** Un décalage de grille est **invisible sur terrain plat**, où toute translation laisse l'erreur inchangée. Le test d'alignement doit donc toujours être restreint aux zones à gradient non nul, les rampes et les bords de structure. Le test antérieur, mené sur l'ensemble du monde, avait dilué le signal et conclu à tort à l'absence de minimum franc.

**Ajouts du 19 octobre 2026.** Modèle d'énergie calibré et validé (C), voir S10. Piège 14 aggravé : le serveur Gazebo apparaît comme `gz sim ...` et échappe aux motifs `ruby` et `gz-sim-server` ; il survit, et PX4 réutilise alors le monde (`gazebo already running world` dans `px4.log`). Les scripts de session tuent désormais `gz[ ]sim` et refusent tout vol sur un monde réutilisé. Le stationnaire de référence de 285.9 W du 17 octobre avait tourné sur un monde réutilisé ; les 24 stationnaires de la calibration, sur monde neuf, donnent 285.8 W.

**Ajouts du 19 octobre 2026, soir.** Premier vol sur une zone de 190 m sous 200 kJ : **7 violations de I1 avec C1 actif**, causées par une latence de décision croissante que C1 facturait à 2 s constantes (`experiments/2026-10-19_big_area`). Correctifs mesurés un par un : couverture rapportée sur la vraie zone, chronométrage complet du cycle, renvois de commandes datés de leur publication (20 renvois, puis 0), latence mesurée dans C1 (avec 5 s de latence injectée sur W1 : violations 2, 2, 1, puis 0, 0, 0 ; `experiments/2026-10-19_decision_latency`), paramètres PX4 remis à zéro avant chaque session (piège 19).

Ouvert, mesuré, non corrigé :

| Problème | Mesure |
|---|---|
| I1 et le retour ne facturent pas la latence des deux dernières décisions, retour et atterrissage | 5 s chacune avec latence injectée, 1.5 kJ pris sur la réserve ; 0.1 à 0.4 s sans |
| `E_land` optimiste en boucle fermée | 3.30 à 4.14 kJ mesurés par mission autonome, contre 3.20 kJ calibrés sur les descentes du script de calibration ; à recalibrer sur les atterrissages de l'exécuteur, avec l'expérience P4 |
| première visite facturée à `dwell_s`, rien n'étant encore mesuré | 15 s réelles pour 2 s facturées avec latence injectée |
| zone confirmée qui oscille entre `CONFIRMED` et `CANDIDATE`, motif « 1 pass, 2 needed » | 5 rétrogradations entre 34 et 88 s sur la démo à 200 kJ ; le nombre de passages d'une zone appariée ne devrait pas décroître |

**À construire.** Registre SLZ dynamique (B), frontières sur la carte (D), C1 (E), C2 (F), exploration autonome (G), campagne d'évaluation (H).

---

## Annexe A. Opérationnel

Cette annexe remplace le document de passation `etat-du-projet-uav.md` du 12 octobre 2026, qui décrivait un monde de validation et des problèmes ouverts depuis dépassés. Ce document ci est désormais la seule référence, et il doit vivre **dans le dépôt**, sous `docs/`, pour être lisible par tout outil travaillant sur le code.

### A.1 Arborescence

`~/ros2_ws` n'est pas un workspace, c'est un conteneur qui abrite deux workspaces indépendants. **Ne jamais lancer `colcon build` depuis `~/ros2_ws`**, uniquement depuis l'intérieur de l'un des deux.

```
~/ros2_ws/
├── droneBefore/          archive de l'ancien stack, consultable, jamais réutilisée
│   └── src/uav_navigation/        branche main, tag archive-pre-rebuild
├── droneAfter/           travail actuel
│   ├── src/
│   │   ├── px4_msgs/              clone PX4, ignoré par git
│   │   └── uav_core/
│   │       ├── package.xml        rclpy, px4_msgs, sensor_msgs, sensor_msgs_py, numpy
│   │       ├── setup.py           déclare les exécutables
│   │       ├── resource/uav_core  fichier vide, marqueur d'index ROS
│   │       └── uav_core/          nodes et modules, voir section 3.3
│   ├── models/x500_slz/model.sdf
│   ├── worlds/validation.sdf              généré, ne pas éditer à la main
│   ├── worlds/validation_truth.npz        généré, vérité terrain exacte
│   ├── px4_config/airframes/4030_gz_x500_slz
│   ├── tools/                             scripts hors ROS
│   ├── docs/                              cette spécification
│   ├── experiments/                       scripts et manifestes des campagnes
│   └── data_survey_vN.npz                 cartes sauvegardées
```

Pour extraire un fichier de l'archive : `git show main:chemin/du/fichier` depuis `droneBefore`.

**Règle sur les campagnes de mesure.** Scripts, README et manifestes vont dans `experiments/<date>_<sujet>/` **à l'intérieur du dépôt**. Les données brutes peuvent rester ignorées par git, à condition qu'un manifeste liste les fichiers, leur taille et leur empreinte. Une campagne stockée hors du dépôt n'est pas reproductible, et la section correspondante du mémoire ne tiendrait pas.

### A.2 Dépendances envers l'arborescence PX4

Trois liens vers l'extérieur du dépôt, à recréer sur toute nouvelle machine, en particulier sur la machine Linux native de la démonstration finale.

```bash
ln -s ~/ros2_ws/droneAfter/models/x500_slz \
      ~/PX4-Autopilot/Tools/simulation/gz/models/x500_slz

ln -s ~/ros2_ws/droneAfter/worlds/validation.sdf \
      ~/PX4-Autopilot/Tools/simulation/gz/worlds/validation.sdf

cp ~/ros2_ws/droneAfter/px4_config/airframes/4030_gz_x500_slz \
   ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
chmod +x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_x500_slz
```

Le modèle et le monde sont des **liens symboliques** pour qu'une modification soit prise en compte immédiatement. L'airframe est une **copie**, parce que PX4 le compile dans sa ROMFS. Il doit en plus être déclaré dans `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, sur une ligne indentée par une tabulation, juste après `4022_gz_x500_lidar_3d` vers la ligne 91. Tout changement de ce fichier force une recompilation de PX4.

### A.3 Séquence de lancement complète

**Nettoyage, systématiquement avant chaque lancement**, pas seulement après un plantage. La moitié des échecs de démarrage viennent d'un processus fantôme de la session précédente.

```bash
pkill -9 -f px4
pkill -9 -f "gz sim"
pkill -9 -f ruby
pkill -9 -f MicroXRCEAgent
pkill -9 -f parameter_bridge
pkill -9 -f rviz2
pkill -9 -f rqt
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson \
      ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
sleep 3
ps aux | grep -E "px4|gz sim|ruby|XRCE|bridge|rviz|rqt" | grep -v grep
```

`ruby` n'est pas une erreur : Gazebo Harmonic lance son serveur par un script Ruby, et c'est typiquement ce processus qui survit. La suppression de `parameters.bson` fait partir PX4 des valeurs de l'airframe à chaque lancement, voir piège 19 ; aucun paramètre de ce fichier n'est réglé à la main.

**Compilation**, après toute modification dans `src/` :

```bash
cd ~/ros2_ws/droneAfter
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uav_core
source install/setup.bash
```

**Onglet 1, simulation.** Attendre `Ready for takeoff!`.

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=validation HEADLESS=1 make px4_sitl gz_x500_slz
```

**Onglet 2, agent XRCE-DDS**, pont PX4 vers ROS.

```bash
MicroXRCEAgent udp4 -p 8888
```

**Onglet 3, pont Gazebo vers ROS.** Le crochet `[` ouvre uniquement le sens Gazebo vers ROS ; un second `@` ouvrirait aussi le sens inverse, inutile pour un nuage de points.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

**Onglet 4, cartographie.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
ros2 run uav_core cloud_world
```

**Onglet 5, vol.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash
ros2 run uav_core survey_mission
```

**Après `Survey complete, landing`**, attendre l'atterrissage, couper `cloud_world` par `Ctrl+C`, puis **immédiatement** mettre la carte à l'abri. `/tmp/elevation_map.npz` est écrasé par n'importe quel lancement ultérieur de `cloud_world`, même sans vol. Une session de validation a déjà été perdue ainsi.

```bash
cp /tmp/elevation_map.npz ~/ros2_ws/droneAfter/data_survey_vN.npz
```

**Validation.**

```bash
cd ~/ros2_ws/droneAfter
python3 tools/validate_terrain.py data_survey_vN.npz
python3 tools/validate_slz.py data_survey_vN.npz
cp /tmp/slz.png /mnt/c/Users/doria/Desktop/
```

**Visualisation du nuage**, facultatif. `Fixed Frame` sur `lidar_link`, puis Add, By topic, `/lidar/points`, Size 0.05.

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

### A.4 Repères, à ne jamais improviser

| Repère | Convention |
|---|---|
| Gazebo, monde | ENU : x est, y nord, z haut |
| Capteur, nuage brut | FLU : x avant, y gauche, z haut |
| PX4, corps | FRD : x avant, y droite, z bas |
| PX4, monde | NED : x nord, y est, z bas |

Conversion du nuage, dans `cloud_world.py` : FLU vers FRD par `pts * [1, -1, -1]`, plus le décalage capteur vers le bas, puis FRD vers NED par rotation du quaternion, plus la position.

Conversion de la vérité terrain, dans les scripts de validation :

```python
truth_ned = -truth_gz.T     # transposition = echange nord/est, signe = haut/bas
zone_ned = zone_gz.T
```

Donc **nord NED = y Gazebo, est NED = x Gazebo**. Ces deux lignes font partie des causes candidates du décalage d'une cellule décrit en P5, et sont à vérifier en priorité par le test sans vol.

### A.5 État git

Branche de travail `dronePropre`. L'ancien stack est archivé sur la branche `main` avec le tag `archive-pre-rebuild` pour `uav_navigation`, et sur `archive-msgs` pour `uav_navigation_msgs`.

Chaque session se termine par un commit et un push, avec un message qui porte la mesure et non seulement l'intention : `Fix footprint element to cover disc, dangerous false accepts 25 -> 0 on v6` plutôt que `Fix footprint`.
