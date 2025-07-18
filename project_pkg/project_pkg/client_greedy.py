import requests
import json
import os

API_URL = "http://192.168.0.179:5050/waypoints"
OUTPUT_FILE = "/home/user/ros2_ws/src/Project_PX4_IIoT/project_pkg/waypoint_file.json"

try:
    num_drones = int(input("Inserisci il numero di droni: "))
    if num_drones <= 0:
        raise ValueError('Il numero di droni deve essere maggiore di 0')
except ValueError as ve:
    print(f'Input non valido: {ve}')
    exit(1)

waypoints = [
    {"lat": 38.1833, "lon": 15.5511, "alt": 30.0},
    {"lat": 38.1826, "lon": 15.5505, "alt": 30.0},
    {"lat": 38.1832, "lon": 15.5496, "alt": 30.0},
    {"lat": 38.1838, "lon": 15.5515, "alt": 30.0},
    {"lat": 38.1842, "lon": 15.5508, "alt": 30.0},
    {"lat": 38.1844, "lon": 15.5502, "alt": 30.0},
    {"lat": 38.1843, "lon": 15.5493, "alt": 30.0},
    {"lat": 38.1836, "lon": 15.5492, "alt": 30.0}
]

body = {
    "waypoints": waypoints,
    "num_drones": num_drones
}

try:
    response = requests.post(API_URL, json=body)
    response.raise_for_status()

    result = response.json()
    with open(OUTPUT_FILE, "w") as f:
        json.dump(result, f, indent=2)
    print(f'File {OUTPUT_FILE} creato/sovrascritto con successo')
except requests.exceptions.RequestException as e:
    print(f'Rochiesta fallita: {e}')
except Exception as e:
    print(f'Errore generico: {e}')
