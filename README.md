# IIoT project
University of Messina A.Y. 2023/2024 | Industrial IoT

Questo progetto ha come obiettivo la realizzazione di un sistema autonomo per la gestione e il controllo di uno o più droni all'interno di un ambiente urbano simulato. Lo scopo principale è quello di simulare missioni di sorveglianza e perlustrazione, distribuendo in modo intelligente i compiti tra i droni disponibili.

I droni, ciascuno con una configurazione parametrica e identificabile in modo univoco, collaborano per esplorare una zona urbana suddividendo i waypoint da raggiungere. La suddivisione è ottimizzata tramite un algoritmo greedy che assegna i punti nel modo più efficiente possibile, minimizzando la distanza percorsa da ciascun drone.

Il sistema si basa su un'infrastruttura software composta da PX4, Gazebo, MAVROS e QGroundControl, tutti configurati all'interno di una macchina virtuale Ubuntu.

## Configurazione degli ambienti

Per la configurazione completa degli ambienti e degli strumenti utilizzati (come PX4, Gazebo, MAVROS, QGroundControl), è possibile consultare la guida completa cliccando su questo link:

👉 [Guida alla configurazione degli ambienti](https://github.com/Andrewww00/Project_PX4_IIoT)

## Avvio del sistema

Per eseguire correttamente una simulazione:

1. **Spostarsi nella directory PX4**:

```
cd ~/PX4-Autopilot
```

2. **Esportare le coordinate della home position** (opzionale ma consigliato per simulazioni realistiche):

```
export PX4_HOME_LAT=45.4642
export PX4_HOME_LON=9.1900
export PX4_HOME_ALT=0.0
```

3. **Lanciare il simulatore PX4 con Gazebo**:

```
HEADLESS=1 make px4_sitl_default gazebo-classic_iris
```
> ℹ️ Il flag `HEADLESS=1` serve per avviare la simulazione in modalità *headless*, ovvero senza interfaccia grafica. Questa modalità è utile soprattutto su server remoti o ambienti senza supporto GUI, poiché permette di risparmiare risorse e avviare più simulazioni in parallelo in modo efficiente.

4. **Avviare QGroundControl**:

Aprire QGroundControl manualmente dall'ambiente desktop oppure digitare il comando appropriato nel terminale, ad esempio:

```
./QGroundControl.AppImage
```


5. **Avviare MAVROS con il nodo di controllo** specificando il numero di droni desiderato:

```
ros2 launch project_pkg drone_launch.py num_drones:=1
```

Se il numero di droni è uguale ad 1, in alternativa è possibile utilizzare i seguenti comandi:
**Per avviare MAVROS**:

```
ros2 launch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14580\"
```
**Per avviare il nodo ROS 2**:

```
ros2 run project_pkg swarm --ros-args -p waypoint_file:="src/Project_PX4_IIoT/project_pkg/waypoint_file.json"
```
## Funzionalità principali

- Simulazione di missioni di sorveglianza in ambienti urbani realistici
- Supporto per un numero arbitrario di droni
- Gestione autonoma e parallela dei droni
- Suddivisione automatica dei waypoint tra i droni
- Ottimizzazione della perlustrazione tramite algoritmo greedy

## Requisiti

- Ubuntu 22.04+
- ROS 2 (Humble o superiore)
- PX4 Autopilot
- Gazebo Classic
- MAVROS
- QGroundControl (opzionale, per monitoraggio grafico)
