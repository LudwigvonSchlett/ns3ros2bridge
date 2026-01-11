#!/usr/bin/env python

# Constantes
MTU = 15000   # Maximum Transmission Unit pour Ethernet frame
MODE = "gpu"  # gpu cpu vm
RANDOM_SEED = 0  # Graine pour le tm et random
# Durée entre chaque groupe de packet
INTERVAL = 1
# Communication avec ns3
CONTROL_NODE_IP = "10.0.0.1"
VEH_NODE_PREFIX = "10.0."
VEH_NODE_SUFFIX = ".1"
NS3_PORT = 12000
# Ports pour carla
CARLA_PORT = 2000
TM_PORT = 8000

# Varibles
# Variables de simulation
carla_sim = "carla"  # carla sim
node = None
nb_nodes = 0   # nombre de nodes et donc de voitures dans la simulation
number_message_sent = 0  # Pour les messages s'envoyant via tap1,2,3,...
number_message_received = 0  # Pour les messages s'envoyant via tap1,2,3,...
simulation_duration = 0
# Listes des vehicules pour carla
vehicles = []
# Thread et essentiels pour la sychronisation
position_listener_thread = None
comunication_nodes_thread = None
listen_tap_devices_thread = None
stop_state = False
error_state = False
