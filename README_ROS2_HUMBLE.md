# Fabtino ROS 2 Humble + Webots system

Target OS: Ubuntu 22.04 / ROS 2 Humble.

## Static evidence, denied zones and deliveries (v8)

During live mapping, occupied cells are promoted only after fixed world-frame
returns have been observed for **strictly more than 3 seconds**. The anchor
does not follow a moving object. Position tolerance is one quarter of a cell,
capped at 2 cm; a capture gap above 0.45 s restarts the observation interval.
Duplicate, older and pre-reset scans provide no evidence. Free-space rays
clear departed obstacles and their evidence. Imported known maps stay fixed.
The filter is a spatial/temporal test: an object that stays still long enough
is treated as static until free-space observations show it has left.

Current LiDAR remains visible in cyan (`/fabtino/live_pointcloud`) and immediately
blocks navigation through `/fabtino/navigation_map`, even before confirmation.
Transient obstacle cells expire after 0.5 s without a return. The exported
occupancy map and yellow/3D static layers contain only the confirmed raster.
The 2D renderer caches that raster between map updates so larger imports do
not delay telemetry, zone confirmations or interaction.
Mapping parameters: `static_persistence_s=3`, `static_observation_gap_s=0.45`,
`dynamic_obstacle_lifetime_s=0.5`.

In **Denied zones**, enter a unique name and radius, press **Place circle on
map**, then click its centre. Cancel or Escape abandons placement. Red circles
appear in 2D and 3D. Saved circles have stable IDs and are acknowledged by the
mission manager. A* checks the expanded circles and exact route segments;
local following shortens lookahead to avoid cutting corners; motor safety
checks the swept navigation command. Each goal carries the current zone
configuration and revision; delayed older configuration messages cannot
overwrite it during navigation. The default centre clearance is 0.60 m
(0.52 m robot footprint plus 0.08 m margin). Manual driving remains available
for recovery and cancels an active delivery. Navigation goals inside a circle
or its clearance are rejected.

In **Delivery**, pick the base on the map (heading 0 degrees), or use the
robot's current position and heading. Choose a destination zone and wait time,
then **Add stop to queue**. Stops run in the listed order, including repeated
destinations. The robot faces the circle at the closest reachable approach
cell outside all exclusions and obstacles, with an additional 0.12 m arrival
buffer. Delays start after the existing arrival controller confirms a settled
stop. Return to base is enabled by default; it can be disabled. **Cancel
delivery**, Stop Nav, a new goal, map operations and manual movement stop the
queue; it cannot resume itself after cancellation. An unreachable destination
fails the queue and leaves the robot stopped. **Return to base** is also
available as an independent action.

PNG export embeds UTF-8 zone/base metadata without painting circles into the
occupancy pixels. JSON export stores exact cells, scale, zones and base.
**Load Known Map** accepts either format and restores annotations only after
successful localization. Plain occupancy images have no annotations and
require their original scale. Reset Odom clears zones and base because the
coordinate frame changes. Clear Map and live mapping keep them in the same
frame. The launch saves configuration atomically at
`~/.local/share/fabtino/navigation.json`; the delivery queue never resumes
automatically on restart. Map import and odometry reset also wait for the
mission manager's annotation confirmation.

Run `python3 tests/run_validation.py --package all --browser` for isolated
package tests, actual TCP/WebSocket tests and Chrome workflows. New cases cover
the strict threshold, moving objects, robot transforms, stale scans, immediate
obstacles, footprint/segment constraints, safe departure beside a circle,
reachability, task waits, failures, cancellation, persistence and PNG/JSON
round trips. Reports are in `tests/results/`; browser captures include
`features-desktop.png` and `features-mobile.png`.

These portable tests substitute ROS topics and Webots physics explicitly.
`tests/pipeline/test_delivery_network.py` disables scan-matcher corrections to
isolate navigation on ideal wheel devices; the Chrome workflow keeps the
production localization defaults and imports a known map. Genuine DDS/Webots
verification requires the Ubuntu runtime:

```bash
python3 tests/live_pipeline.py --delivery-json tests/fixtures/delivery.json
# Optional: also test a known map, including its saved zone/base metadata.
python3 tests/live_pipeline.py --map-json saved-map.json --delivery-json delivery-test.json
```

Adjust the fixture's positions to the active world and reset frame before
running it. Live test delays must be 0–3 seconds. The live script measures
rates/freshness and tests deliveries, waits, circle clearance and base return;
it reports unavailable when ROS is absent and never substitutes a live runtime.

## Command, reset and map operations (v7)

Manual driving and navigation restart LiDAR acquisition when it was disabled.
The TCP bridge sends scan/reset actions independently of the latest motor
command, so a stream of motor updates cannot discard those actions. Capture
timestamps stay intact; delayed motor commands expire. Reconnecting after a
Webots simulation restart also restarts the scan publication interval.

Clear Map, Reset Odom, image import, live mapping and map settings now use
`/viewer/request` and `/viewer/operation_status`, with a request ID and an
explicit completion or error. The browser waits for confirmation and displays
the reason for failure. A timeout cancels a pending operation and permits retry.
Movement stays stopped during an operation; it requires a new command afterward.

Clear Map clears the ROS grid and queued scans, publishes an empty map/point
cloud immediately, and starts a new map generation. New scans rebuild the map.
Old map, cloud and reset-pose messages are excluded by acquisition timestamps.
Reset Odom resets the EKF, wheel baseline, IMU yaw reference and mapper together;
it completes only after fresh wheel and IMU measurements. The `/fabtino/reset`
service requests the same estimator/map reset.

Import accepts a square occupancy image at its original resolution and radius:
black means occupied, white means free, gray or transparent means unknown. The
image is flipped into the map's positive-Y convention without interpolation.
The localization node matches a fresh stationary LiDAR scan in a worker thread;
the mapper installs the static grid after successful localization. An unsuitable
image returns an error while retaining the previous map. Live Map clears the
static map and resumes mapping. Slider changes now update the ROS grid too.

### Reproducible validation

Install the test dependencies once, then run each package suite and the complete
portable pipeline:

```bash
python3 -m pip install -r requirements_tests.txt
python3 tests/run_validation.py --package all
# One package only:
python3 tests/run_validation.py --package fabtino_mapping

# Chrome must be installed. Install Playwright once:
npm install --prefix .test_runs/ui playwright
# Linux, if needed: PYTHON=python3
PYTHON=python3 python3 tests/run_validation.py --package all --browser
```

There are isolated suites for all eight ROS packages, the Webots controller,
the frontend and the network pipeline. The package `test/` entries also support:

```bash
cd ros_ws
colcon test --packages-select fabtino_mapping --event-handlers console_direct+
colcon test-result --verbose
```

The portable tests replace ROS DDS and Webots devices explicitly. The network
test uses real TCP and WebSocket connections, and measures sensor age, viewer
rate, cancellation, lost heartbeats and reconnect delay. The Chrome pipeline
uses the production HTML and JavaScript through the real gateway and controller;
it covers movement, target arrival, clears/resets, and PNG import after moving.
Results are written to `tests/results/package_validation.json` and
`tests/results/realtime_pipeline_validation.json`.

The live test imports genuine `rclpy` and uses the running ROS/Webots system.
It also checks physical movement with diagnostic ground truth. On Ubuntu:

```bash
bash build_ros2.sh
bash run_full_project.sh
# In another terminal:
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
python3 tests/live_pipeline.py
# Include live map import with a known map JSON containing resolution, radius
# and a flat grid of -1/0/100 values:
python3 tests/live_pipeline.py --map-json /path/to/known-map.json
```

The live test sends short movement commands, then stops the robot on exit. It
reports missing ROS as `unavailable` with exit code 2, and failures with exit
code 1. Map import is marked as not run when no known map is supplied.
Live DDS/Webots physics and `colcon` could not be executed in this Windows
workspace: the available Ubuntu installation has no ROS installation. Portable
passes do not establish physical calibration or performance on the target VM.

Restart both ROS and Webots after rebuilding, then reload the viewer with
**Ctrl+F5** to load v7.

## Runtime architecture

Webots -> `fabtino_webots_bridge_controller` (TCP 127.0.0.1:8766) -> `fabtino_webots_bridge` -> ROS topics -> localization -> mapping -> global A* -> local planner -> safety supervisor -> `/fabtino/cmd_vel` -> bridge -> Webots.

The browser is isolated behind `fabtino_viewer` on WebSocket port 8765. It is an operator/visualization client only.

## Recul contre un mur : position affichee (v6)

Le superviseur protege maintenant aussi le recul : un obstacle dans le couloir
arriere coupe la commande de marche arriere, manuelle ou autonome. Les valeurs
par defaut sont `rear_stop_distance_m: 0.65` et `rear_half_width_m: 0.35`.
Avancer pour s'eloigner du mur reste possible. Le site affiche « Recul bloque »
et indique cette sortie.

La localisation ne compte plus une translation des roues qui entre dans un
obstacle proche observe au LiDAR. Cela empeche les roues qui patinent contre
un mur de faire reculer indefiniment la pose affichee et de deplacer ce mur
dans la carte. Les angles d'encodeur sont consommes meme quand l'increment
est rejete : repartir en avant ne rejoue pas les rotations rejetees.
Les parametres ROS de localisation sont
`translation_collision_distance_m: 0.50`,
`translation_collision_half_width_m: 0.35` et
`collision_scan_timeout_s: 0.30`. Cette borne conservative utilise une mesure
LiDAR valide et recente dans le sens du mouvement; un obstacle lateral, une
mesure invalide ou un ancien scan ne figent pas l'odometrie.
La position reelle Webots reste reservee au diagnostic.

Le test `test_wall_contact.py` maintient des encodeurs annoncant 0.5 m/s de
recul pendant 20 secondes devant un mur fixe : la pose envoyee au navigateur
et les points du mur restent fixes, puis une avance de 1 cm produit seulement
1 cm de deplacement. Il utilise les vrais callbacks avec transport simule;
il ne remplace pas un essai de physique Webots/ROS. Executer :

```bash
python3 -m unittest discover -s tests -p test_wall_contact.py -v
python3 -m unittest discover -s tests -p test_safety_arbitration.py -v
```

Pour appliquer : `bash build_ros2.sh`, relancer ROS et Webots avec
`bash run_full_project.sh`, puis **Ctrl+F5** dans le navigateur. Repartir
d'une carte neuve si l'ancienne position a deja derive : ce correctif ne
relocalise pas une carte deja faussee.

## Commandes et boutons du navigateur (v5)

Maintenir **↶ / ↷** ou **← / →** pour tourner sur place vers la gauche ou
la droite; relacher pour envoyer une vitesse nulle. Le maintien est compatible
avec la souris, le tactile et le clavier. Quitter la page, perdre le focus ou
perdre la connexion efface les directions enfoncees. Le panneau indique les
commandes envoyees et explique les arrets dus aux capteurs ou aux obstacles.
Le superviseur ignore maintenant les mesures hors des limites `range_min` /
`range_max` du LiDAR; les obstacles valides proches continuent de bloquer la rotation.

Les boutons **2D MAP / 3D VIEW** attendent le chargement de la vue 3D et
respectent un retour en 2D pendant ce chargement. Three.js 0.160.0 et
OrbitControls sont distribues dans `js/vendor/three/`, avec leur licence MIT,
pour fonctionner sans CDN. Si WebGL est indisponible, un message apparait et
la vue 2D reste utilisable. Le zoom et le deplacement de la carte se redessinent
immediatement, meme sans nouvelles donnees du robot. Sur petit ecran, la carte
et ses boutons apparaissent au-dessus des commandes.

**Clear Map** envoie desormais une demande `/viewer/request` au serveur,
et l'etat du bouton LiDAR suit la demande d'activation/desactivation.

Validation reproductible :

```bash
node --test tests/test_navigation_ui.cjs tests/test_map_orientation.cjs
python3 -m unittest discover -s tests -p test_drive_chain.py -v
python3 -m unittest discover -s tests -p test_safety_arbitration.py -v
# Pour les parcours navigateur : Chrome installe et Playwright dans ce dossier.
npm install --prefix .test_runs/ui playwright
node tests/controls_browser.cjs
node tests/navigation_browser.cjs
```

Le parcours navigateur utilise les vrais modules 2D/3D, avec le WebSocket ROS
simule. Le test de commandes utilise les vrais callbacks de la passerelle,
du superviseur et des deux ponts, avec transport et moteurs simules; il ne
mesure pas la physique Webots. Pour appliquer les changements Python sur
Ubuntu/ROS, executer `bash build_ros2.sh`, relancer `bash run_full_project.sh`,
puis recharger la page avec **Ctrl+F5**.

## Masquage angulaire du LiDAR

Le controleur Webots charge `config/robot_parameters.yaml` au demarrage,
depuis la racine du projet (independamment du repertoire de lancement).
Le nouveau parametre est desactive par defaut :

```yaml
lidar:
  masked_angle_ranges: []
```

Exemple pour exclure deux secteurs :

```yaml
lidar:
  masked_angle_ranges:
    - [10, 30]
    - [100, 120]
```

Les bornes sont inclusives, en degres dans le repere local du LiDAR :
0 devant, angles positifs antihoraires. L'indice i correspond exactement a
`-degrees(FOV)/2 + i*degrees(FOV)/(N-1)`; pour N=1, le pas vaut zero.
FOV, N et le nombre de couches proviennent du capteur au demarrage.
Les intervalles negatifs et ponctuels sont acceptes. Les bornes non numeriques,
non finies, inversees ou hors FOV provoquent une erreur; les chevauchements
sont signales puis fusionnes. La comparaison utilise 1e-6 deg de tolerance.

Le masque est precalcule une fois. Chaque acquisition copie le tableau du
capteur, remplace les indices masques par `+inf` et conserve l'ordre et les
N*couches mesures. Le JSON strict transporte ces valeurs sous forme de `null`,
restaure en `+inf` avant publication ROS. Cartographie, localisation et
superviseur ignorent ces mesures; elles ne creent ni point ni rayon libre.
Le transport conserve toutes les couches; la pile ROS actuelle reste 2D et
prend la couche centrale (indice couches//2), sans concatener leurs angles.
Ce changement n'ajoute pas de cartographie 3D.

Apres une modification du YAML, redemarrer Webots. Le fichier n'est pas relu
a chaque tick. La methode `Controller.reload_lidar_mask()` permet un
rechargement explicite par du code : elle recalcule les indices, y compris si
la geometrie change, et leve une erreur sans remplacer le masque precedent
si la nouvelle configuration est invalide. Il n'y a pas de surveillance
automatique du fichier ni de nouvelle commande navigateur.

PyYAML est deja dans `requirements_ros2.txt`. Pour la premiere installation de
ce correctif, reconstruire ROS avec `bash build_ros2.sh` et redemarrer les deux
cotes du pont, car le decodage des mesures `null` a aussi ete adapte.

```bash
python3 -m unittest discover -s tests -p test_lidar_mask.py -v
```

Ces tests utilisent les vrais callbacks avec capteurs/transport simules;
ils ne constituent pas une execution complete Webots/ROS.

## Ground truth policy

`/fabtino/ground_truth/odom` is evaluation-only. It is not subscribed to by localization, mapping, planning, or safety.

## Main topics

`/fabtino/scan`, `/fabtino/imu/data_raw`, `/fabtino/joint_states`, `/fabtino/odometry/filtered`, `/fabtino/map`, `/fabtino/local_costmap`, `/fabtino/pointcloud`, `/navigation/global_path`, `/navigation/cmd_vel`, `/fabtino/cmd_vel`, `/fabtino/ground_truth/odom`.

## Build on Ubuntu 22.04

Install ROS 2 Humble first, source `/opt/ros/humble/setup.bash`, then install Python dependencies with `python3 -m pip install -r requirements_ros2.txt`.

Then:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run Webots

Open `webots/worlds/world_simulation.wbt` (the furnished coffee shop). This is also the default for `run_full_project.sh`. The Fabtino controller is `fabtino_webots_bridge_controller` and exposes TCP 8766. The previous world remains available as `webots/worlds/world_fixed.wbt`.

## Run ROS

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
ros2 launch fabtino_bringup fabtino_sim.launch.py
```

## Run viewer

Serve the repository root or use any static HTTP server, e.g.:

```bash
python3 -m http.server 8000
```

Open `http://127.0.0.1:8000/map_viewer2_modular.html`. The viewer WebSocket is ROS-backed at `ws://127.0.0.1:8765`.

## Manual ROS checks

```bash
ros2 topic hz /fabtino/scan
ros2 topic echo /fabtino/odometry/filtered
ros2 topic echo /fabtino/safety_status
ros2 topic echo /navigation/global_status
ros2 topic echo /navigation/global_path
ros2 topic pub --once /teleop/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

## Navigation rules in this version

- Global A* allows UNKNOWN cells and applies `unknown_cost` as an additive traversal cost.
- Navigation grid is 10 cm cells.
- Obstacles are inflated by the robot radius (52 cm) plus an 8 cm margin, rounded up to grid cells.
- An out-of-map robot or goal stops planning and reports the problem.
- Local planner uses route-relative lookahead and scales forward speed with heading error. Large turns use a bounded alignment phase with angular damping.
- Safety has a decision-age watchdog separate from transport freshness.

## Navigation : arret et orientation finale

Apres cette mise a jour, reconstruire puis relancer les noeuds sur Ubuntu 22.04
(depuis la racine du projet, apres avoir arrete l'ancienne execution) :

```bash
bash build_ros2.sh
bash run_full_project.sh
```

Recharger le navigateur avec Ctrl+F5. Dans **Autonomous Navigation**, saisir
X et Y en metres, puis **Final orientation** en degres : par exemple X=10,
Y=0, angle=90. Cliquer sur **Navigate to Coordinates**.
Un clic sur la carte ouvre maintenant **Orientation a l'arrivee** : saisir
l'angle puis **Demarrer**, ou cocher **Sans orientation finale**.
**Annuler** et Echap ferment la fenetre sans envoyer de nouvel objectif.
L'objectif precedent, s'il existe, reste actif tant qu'un autre n'est pas confirme.

- Les coordonnees et l'angle sont absolus dans le repere de la carte :
  0 deg = +X, 90 deg = +Y, 180 deg = -X. Les angles positifs sont antihoraires.
- Un angle vide demande uniquement la position, sans rotation finale imposee.
- Le robot ralentit, atteint la position, ajuste son orientation si demandee,
  puis confirme l'immobilite mesuree pendant 0,40 s avant de verrouiller l'arret.
- **Stop Nav** annule l'objectif et le trajet. Le bouton carre **Stop** annule
  aussi la navigation. Une nouvelle commande d'objectif permet de repartir.
- Un ancien trajet ou une ancienne notification d'arrivee ne peut pas
  reactiver/terminer un autre objectif : chaque objectif possede un identifiant.

Les parametres ROS du noeud `local_planner` se reglent dans
`ros_ws/src/fabtino_bringup/launch/fabtino_sim.launch.py` :

| Parametre | Valeur initiale | Role |
| --- | --- | --- |
| `goal_tolerance_m` | 0.05 | Precision finale demandee : 5 cm |
| `yaw_tolerance_rad` | 0.05235987756 | Tolerance angulaire : 3 deg |
| `final_max_angular_rps` | 0.35 | Vitesse maximale de rotation finale |
| `settle_time_s` | 0.40 | Duree de confirmation de la pose et de l'immobilite |
| `yaw_hysteresis_rad` | 0.03490658504 | Marge interne de freinage, bornee a la moitie de la tolerance |
| `final_approach_distance_m` | 0.40 | Debut de l'approche lente sur le dernier segment libre |
| `final_linear_mps` | 0.08 | Vitesse maximale de l'approche finale |
| `final_heading_kp` / `final_heading_kd` | 1.2 / 0.35 | Gain d'angle / amortissement par la vitesse mesuree |
| `stopped_linear_mps` | 0.01 | Seuil de vitesse lineaire pour confirmer l'arret |
| `stopped_angular_rps` | 0.03 | Seuil de vitesse angulaire pour confirmer l'arret |
| `pose_timeout_s` | 0.30 | Age maximal du retour de position |
| `alignment_no_progress_s` | 5.0 | Arret en echec si l'erreur d'angle ne diminue pas |
| `alignment_timeout_s` | 30.0 | Duree maximale de la tentative d'orientation finale |
| `path_max_angular_rps` | 0.70 | Limite de rotation pour rejoindre le trajet |
| `path_heading_kd` | 0.35 | Amortissement pendant le trajet |
| `path_turn_no_progress_s` / `path_turn_timeout_s` | 5.0 / 20.0 | Arret si l'orientation ne progresse pas / dure trop longtemps |
| `movement_no_progress_s` | 15.0 | Arret si le robot n'avance pas d'au moins 3 cm |

Ces valeurs sont des points de depart a valider avec l'odometrie Webots.
L'arrivee est evaluee sur la pose estimee, pas sur la position ground truth.
Apres `goal_reached`, de petites variations de localisation ne relancent pas
la commande. Une correction exige un nouvel objectif.

Le controleur vise une marge interne de 2,5 cm avant de freiner, puis exige
que la pose reste dans les 5 cm demandes. Le freinage angulaire commence
vers 1,5 deg d'erreur; la tolerance finale reste de 3 deg. Ces marges internes
absorbent le bruit et l'inertie sans elargir la precision annoncee. Si la
position sort des 5 cm pendant la confirmation, une nouvelle approche est
necessaire. La vitesse reelle estimee par l'odometrie doit aussi rester sous
les seuils ci-dessus pendant 0,4 s : envoyer zero aux moteurs ne suffit plus
pour annoncer `goal_reached`. Un retour de position trop ancien interrompt
la confirmation et commande l'arret.

Le superviseur expire les commandes manuelles apres 0,30 s sans rafraichissement
(`teleop_timeout_s`). Un nouvel objectif ou Stop libere l'ancienne commande
manuelle. Apres succes/echec de la mission, une ancienne commande de navigation
recue en retard ne peut plus relancer les moteurs. La perte du focus du navigateur
relache aussi les touches de conduite.

Si l'orientation ne converge pas, le controleur publie `goal_failed`, coupe
les vitesses et termine la mission. Il ne declare pas une fausse arrivee.
Les raisons sont `final_heading_no_progress` (pas d'amelioration d'au moins
1 deg pendant 5 s) et `final_heading_timeout` (tentative de plus de 30 s).
Une nouvelle commande est necessaire pour repartir.

Pour verifier la version effectivement executee sur Ubuntu, apres avoir
reconstruit et redemarre les noeuds :

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
ros2 topic echo /navigation/local_status
```

La sortie doit contenir `controller_version: arrival-v4` dans le JSON.
`distance_m`, `yaw_error_deg`, `position_reached`, `command_linear_mps` et
`command_angular_rps`, `measured_linear_mps`, `measured_angular_rps` et
`pose_age_s` sont actualises toutes les 0,5 s, meme sans changement
d'etat. Une erreur d'angle `null` indique qu'aucune orientation finale n'est
imposee (ou qu'aucune pose/objectif n'est encore disponible). En cas de
rotation persistante, conserver cette sortie et celle de `/fabtino/cmd_vel`
pour distinguer une commande du planificateur d'une autre source de mouvement.
Le topic `/fabtino/safety_status` expose aussi `command_source`,
`output_linear_mps` et `output_angular_rps`, c'est-a-dire la commande finalement
selectionnee pour les moteurs.

Pour tester sans l'interface :

```bash
source /opt/ros/humble/setup.bash
source ros_ws/install/setup.bash
ros2 topic pub --once /navigation/request std_msgs/msg/String "{data: '{\"type\":\"goal\",\"x\":10.0,\"y\":0.0,\"yaw_deg\":90.0}'}"
ros2 topic echo /navigation/mission_status
# Dans un autre terminal source de la meme facon, verifier les vitesses nulles a l'arrivee :
ros2 topic echo /fabtino/cmd_vel
# Annuler une navigation :
ros2 topic pub --once /navigation/request std_msgs/msg/String "{data: '{\"type\":\"stop\"}'}"
```

Le topic standard `/navigation/global_path` reste disponible pour visualiser
le trajet. Le controleur utilise `/navigation/route` (JSON avec `goal_id` et
`points`) pour rejeter les anciens trajets. Envoyer les commandes utilisateur
sur `/navigation/request`, pas sur ces topics internes.

Tests de regression executables sans ROS (NumPy requis pour le second) :

```bash
python3 -m unittest discover -s tests -p test_goal_control.py -v
python3 -m unittest discover -s tests -p test_navigation_flow.py -v
node --test tests/test_navigation_ui.cjs
```

Les tests couvrent le controle cinematique et les callbacks avec un transport
ROS simule. Ils ne remplacent pas un essai reel ROS 2 / Webots : tester une
arrivee sans angle, les angles 80/90/180 deg, Stop pendant le deplacement et
pendant la rotation, puis un nouvel objectif apres l'arret.

## Nouveau cafe et navigation v4

Les six fichiers fournis sont conserves sans modification dans
`webots/worlds/source_worlds`. Le monde integre `world_simulation.wbt`
contient le cafe, le robot et ses capteurs; le maillage OBJ est a cote.
Les chemins locaux sortant du projet et les etats caches `hidden` ont ete
retires des mondes executables. Le point de vue initial montre le cafe.
La photo fournie est un apercu 3D, pas une grille d'occupation a charger
avec **Load Known Map** : utiliser la carte produite par le LiDAR.

Le rayon de roue est aligne a 0,10 m dans le pont Webots, la localisation et
les configurations, d'apres le cylindre de collision du
[Fabtino R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/rec/fabtino/protos/Fabtino.proto).
Les deux materiaux de contact mecanum sont configures selon le
[guide officiel Fabtino](https://www.cyberbotics.com/doc/guide/fabtino?version=R2023a).
Ces corrections de configuration ne constituent pas une mesure de calibration physique.

Un test a reproduit des commandes alternant +1/-1 rad/s quand l'objectif
etait derriere le robot, avec seulement +/-0,002 rad de bruit sur le cap.
Le sens de demi-tour est maintenant conserve autour de +/-180 deg, au depart
comme a l'arrivee. Un angle final de 90 deg n'est pas applique au depart :
le robot s'oriente d'abord vers le trajet, puis ajuste l'angle a destination.
Une rotation necessaire reste donc normale, mais une rotation sans progres
termine en echec avec commande nulle. Idem si la translation reste bloquee.

Le superviseur verifie maintenant un couloir de 70 cm de largeur devant le
robot (jusqu'a 65 cm du centre), et bloque la rotation si un retour LiDAR
se trouve a moins de 52 cm du centre. Cela complete l'inflation A* pour les
pieds de tables/chaises. Ce n'est pas une garantie de collision zero : le
LiDAR observe une tranche horizontale, les obstacles hors de cette tranche
et les contacts mecanum exigent toujours une validation Webots.

Resultats v4 : **324/324 scenarios dynamiques**, **72 tests Python**,
**11 tests JavaScript** et un parcours dans Chrome (clic, saisie, annulation,
clavier, affichage mobile). Le banc inclut maintenant les departs a +/-180 deg
avec retards et inertie. Rapport : `tests/results/navigation_v4_validation.json`.
Le navigateur utilise le vrai HTML/JS 2D, avec transport ROS et vue 3D substitues.
Les essais dynamiques et de callbacks n'executent ni DDS ni la physique Webots.
Le moteur Webots local avait echoue au demarrage avec `0xC000007B`.

Pour conserver un autre monde au lancement :
`FABTINO_WORLD="$PWD/webots/worlds/world_fixed.wbt" bash run_full_project.sh`.
Apres cette mise a jour, reconstruire avec `bash build_ros2.sh`, relancer
ROS et Webots avec `bash run_full_project.sh`, puis recharger le navigateur.
Repartir d'une carte LiDAR neuve pour ce nouvel environnement et cette geometrie.

## Validation historique de l'arrivee v3

Les resultats reproductibles sont dans `tests/results/arrival_validation.json`.
Deux defauts ont ete reproduits avant correction : une ancienne rotation manuelle
restait prioritaire sur le zero de navigation; et une arrivee pouvait etre declaree
alors que le robot avait encore une vitesse non nulle. Les seuils elargis de la
version precedente permettaient aussi de terminer hors de la precision demandee.

La version precedente du banc `tests/arrival_dynamics.py` comparait 219 scenarios avec des trajets de 1 m
et 9 m, plusieurs orientations, des retards de 0 a 0,2 s, une inertie de 0,1 a
0,6 s et un bruit de position allant jusqu'a 5 mm par axe. Pour une exigence de
5 cm / 3 deg et un robot immobile a l'annonce du succes :

- Version precedente : 42/219 scenarios satisfont tous les criteres.
- Version corrigee : 219/219; erreur maximale apres stabilisation de 3,164 cm
  et 2,883 deg; aucune commande de mouvement apres succes.
- Cinq scenarios supplementaires font fonctionner ensemble les vrais callbacks
  de mission, A*, controle local et superviseur, avec un modele moteur synthetique.

Cette verification historique comprend 58 tests Python et 9 tests JavaScript
reussis. La validation v7 ci-dessus ajoute des suites portables pour tous les
paquets, y compris `test_mapping_runtime` et `test_viewer_pointcloud`, avec
des doubles ROS explicites quand la suite portable est executee.
Le banc dynamique renvoie un code d'echec si un scenario ne satisfait pas les criteres.

Ce banc est un modele de test, pas Webots. Il ne simule pas les contacts des
roues mecanum ni un biais de calibration ou une erreur de carte. Les tolerances
ROS portent sur la pose estimee; elles ne garantissent pas a elles seules une
precision physique identique. ROS/DDS n'a pas ete execute ici. Une tentative
avec le Webots Windows installe a echoue au demarrage du moteur avec le code
`0xC000007B`, avant le chargement du monde de test.

```bash
python3 tests/arrival_dynamics.py /tmp/arrival-results.json
python3 -m unittest discover -s tests -p test_arrival_chain.py -v
python3 -m unittest discover -s tests -p test_safety_arbitration.py -v
```

## Carte qui tourne pendant une rotation du robot

La chaine active conserve maintenant l'heure d'acquisition fournie par le
controleur Webots (`wall_time_ns`). Les roues et l'IMU d'un meme paquet sont
fusionnees ensemble, quel que soit l'ordre de leurs callbacks ROS. L'odometrie
publiee garde cette heure, meme lorsqu'un timer republie la derniere pose.
Ce chemin suppose Webots et ROS sur le meme ordinateur, avec l'horloge systeme
(configuration du lancement fourni, sans `use_sim_time`).

Une comparaison relative d'horodatages Unix ecrasait auparavant l'historique
des poses : a environ 1,79 milliard de secondes, une tolerance relative de
1e-9 considere presque 1,8 seconde de poses comme identiques. Le mapper ne
remplace maintenant que les echantillons portant exactement le meme temps.
Il utilise une pose exacte ou une interpolation entre deux poses proches;
il attend une pose manquante et rejette les scans trop anciens ou les grandes
lacunes, sans reutiliser arbitrairement le dernier angle connu. Le parametre
`pose_match_tolerance_s` borne desormais l'intervalle d'interpolation
(0,10 s dans le lancement), pas une extrapolation vers la derniere pose.

Le montage de l'IMU dans `webots/worlds/world_fixed.wbt` est aligne sur les
axes du robot, du gyroscope et de l'accelerometre. La convention ENU et
l'orientation propre du capteur sont documentees dans la
[reference Webots R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/inertialunit.md).
Le signe de rotation des symboles du robot en 2D/3D est egalement corrige;
les points et les cellules de la carte restent dans le repere fixe `map`.

Pour appliquer : arreter ROS **et Webots**, reconstruire avec
`bash build_ros2.sh`, puis relancer avec `bash run_full_project.sh` pour
recharger aussi le monde modifie. Recharger le navigateur avec Ctrl+F5.
Repartir d'une carte neuve : les anciens points deja mal projetes ne peuvent
pas etre corriges par ce changement.

Verifier devant un mur : scanner a l'arret, tourner de +90 deg puis de
-90 deg, avancer et tourner de nouveau. Le mur doit garder sa position dans
la carte; seul le robot change de position/orientation. Tester ensuite un
objectif avec orientation finale et Stop Nav.

```bash
python3 -m unittest discover -s tests -p test_rotation_mapping.py -v
node --test tests/test_map_orientation.cjs
```

Ces regressions reproduisent des horodatages Unix, des callbacks inverses,
des retards de transport et une rotation devant un point de mur fixe.
La verification avec les vrais capteurs Webots reste a effectuer sur Ubuntu.

## Validation note

ROS 2 is not installed in the environment used to assemble this archive, so `colcon build` and live Webots↔ROS testing must be run on the Ubuntu 22.04 / ROS 2 Humble VM. Python sources were syntax-checked.

## Important scope note

This is a complete ROS package conversion and integration baseline, but it still uses the project's lightweight custom Python EKF/A* code rather than Nav2/robot_localization. Hardware-specific Webots calibration and the exact Fabtino mecanum sign conventions should be validated in Webots before treating the simulation as regression-approved.
