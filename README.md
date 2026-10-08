# Simulazione AutoSweep per Eureka

![Gazebo: mondo coverage1 con la spazzatrice](images/gazebo_coverage1.png)

![RViz: LiDAR e camera nel mondo coverage1](images/rviz_coverage1.png)

Simulazione ROS 2 Jazzy e Gazebo Harmonic della spazzatrice autonoma Eureka.

Il milestone attuale comprende il modello dei sensori, una trasmissione a triciclo
provvisoria, `ros2_control`, l'odometria ricavata dalle ruote e la teleoperazione
temporanea da tastiera.

## Componenti implementati

- Due LiDAR Hesai JT128 simulati.
- Una camera RGB-D Intel RealSense D456 simulata.
- Una ruota anteriore sterzante e motrice provvisoria.
- Due ruote posteriori passive.
- `joint_state_broadcaster` e `tricycle_controller`.
- `/odom` ricavata dalle ruote e TF `odom -> base_footprint`.
- `/odom_path` con la cronologia dell'odometria visualizzata come linea in RViz.
- Interfaccia `/cmd_vel` condivisa basata su `geometry_msgs/msg/TwistStamped`.
- Teleoperazione temporanea da tastiera.
- Mondo `coverage1_world.sdf` con la mappa statica `coverage1_map` ricostruita
  da `finer_mapping`.

## Compilazione

Eseguire la compilazione dalla radice del repository:

```bash
source /opt/ros/jazzy/setup.bash

colcon build \
  --base-paths autosweep_description autosweep_gazebo \
  --symlink-install \
  --packages-select autosweep_description autosweep_gazebo \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Avvio della simulazione

Nel primo terminale, dalla radice del repository:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch autosweep_gazebo simulation_launcher.py use_sim_time:=true
```

Questo avvia Gazebo, il bridge ROS/Gazebo, `robot_state_publisher`, entrambi i
controller `ros2_control`, il convertitore da odometria a path e RViz.

RViz usa `odom` come frame fisso e visualizza `/odom_path` con il display
`Path`. Il convertitore aggiunge una posa ogni 0,05 m oppure ogni 0,05 rad e
conserva al massimo 5000 pose, evitando di accumulare tutti i messaggi
odometrici pubblicati mentre il robot è fermo.

## Mondo di simulazione

Il mondo predefinito è `autosweep_gazebo/worlds/coverage1_world.sdf`, che include
il modello `autosweep_gazebo/models/coverage1_map` (mesh visuale, texture e mesh
di collisione). Il launcher aggiunge `models/` a `GZ_SIM_RESOURCE_PATH`; un altro
mondo si seleziona con l'argomento `world:=<file>.sdf`.

Per aggiornare la mappa, copiare `coverage1_map/` e `coverage1_world.sdf` da
`finer_mapping/outputs/coverage1/gazebo/` e, nel file del mondo:

- rinominare il mondo in `sweeper_world` (nome usato dal launcher per lo spawn);
- sostituire l'URI assoluto `file:///...` con `model://coverage1_map`.

Poi ricompilare.

## Controllo da tastiera

Eseguire il nodo per la tastiera in un secondo terminale. Il terminale deve
rimanere attivo perché il nodo legge direttamente i tasti premuti.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_footprint \
  -p use_sim_time:=true \
  -p speed:=0.10 \
  -p turn:=0.05
```

Tasti verificati:

| Tasto | Comando |
| --- | --- |
| `u` | Avanti e a sinistra |
| `i` | Avanti |
| `o` | Avanti e a destra |
| `,` | Retromarcia |
| `k` o Spazio | Arresto |

Una pressione pubblica un comando. Mantenendo premuto un tasto di movimento si
utilizza normalmente la ripetizione della tastiera del sistema operativo. Se non
arrivano nuovi comandi, il controller arresta il veicolo dopo 500 ms. Premere
`Ctrl+C` per terminare il nodo della tastiera; prima di uscire pubblica un ultimo
comando con velocità zero.

La convenzione delle coordinate ROS usa `x` in avanti, `y` verso sinistra e `z`
verso l'alto. Un valore positivo di `angular.z` fa curvare a sinistra.

## Principali interfacce ROS

| Topic | Tipo | Scopo |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Comando della base mobile |
| `/odom` | `nav_msgs/msg/Odometry` | Odometria ricavata dalle ruote |
| `/odom_path` | `nav_msgs/msg/Path` | Cronologia delle pose per RViz |
| `/joint_states` | `sensor_msgs/msg/JointState` | Feedback dei giunti mobili |
| `/clock` | `rosgraph_msgs/msg/Clock` | Orologio della simulazione Gazebo |

## Limitazioni attuali (TODO)

- Geometria della trasmissione, masse, inerzie, limiti di sterzata, attrito e
  parametri di contatto sono provvisori e richiederanno un aggiornamento una volta
  disponibili i parametri del prototipo.
- Le ruote e i giunti di sterzata sono articolati fisicamente, ma le rispettive
  visuali mobili non sono ancora state separate dalla mesh monolitica del telaio.
- Il controllo da tastiera è temporaneo. L'integrazione del joystick verrà
  completata quando sarà disponibile l'hardware previsto e sarà possibile
  misurarne assi e pulsanti.
- I sensori di prossimità verranno aggiunti quando saranno disponibili le loro
  specifiche.
