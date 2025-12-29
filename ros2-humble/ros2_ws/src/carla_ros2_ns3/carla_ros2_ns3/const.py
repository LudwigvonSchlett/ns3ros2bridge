#!/usr/bin/env python

# Constantes
MTU = 15000   # Maximum Transmission Unit pour Ethernet frame
MODE = "gpu"  # gpu cpu vm

# Varibles
# Variables de simulation
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
