#!/usr/bin/env python

import random
import time
import sys
import select
import threading

import rclpy

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.mock_sim import (
    init_mock
)
from carla_ros2_ns3.lib.carla_sim import (
    get_all_mobility,
    get_all_position,
    get_position,
    stop_vehicules
)
from carla_ros2_ns3.lib.net import (
    check_message,
    print_udp,
    tap_sender,
    tap_sender_control,
    connect_tap_device
)
from carla_ros2_ns3.lib.ros import (
    create_node,
    destroy_node,
    inflog,
    errlog
)


def main():
    """Initialise le noeud principal et lance le programme."""
    rclpy.init(args=sys.argv)
    try:
        node_name = "carla_ros2_ns3"
        create_node(node_name)
        socket_tap0 = connect_tap_device("tap0")
        tap_sender_control("hello_ROS2")
        control_node_listener(socket_tap0)

    except KeyboardInterrupt:
        pass

    finally:
        destroy_node()


def control_node_listener(socket_tap0):
    """Permet d'ecouter ce que recoit tap0, noeud de controle."""

    while rclpy.ok():
        try:

            packet = socket_tap0.recv(cst.MTU)

            # Récupérer l'adresse IP de destination du paquet
            dest_ip = packet[30:34]

            # Convertir l'adresse IP de destination en une chaine lisible
            dest_ip_str = ".".join(str(byte) for byte in dest_ip)

            # Vérifiez si le paquet est destiné
            # à votre propre adresse TAP pour l'ignorer

            if check_message(packet) and dest_ip_str != '10.0.0.2':
                inflog("tap0 received a packet from control node")
                print_udp(packet)
                message = (packet[42:].decode()).rstrip("\n")
                inflog(f"Received packet (decoded): {message}")
                msg_split = message.split(" ")
                response_command = msg_split[0]

                if response_command == "hello_NS3":

                    inflog("Requesting simulation duration")
                    tap_sender_control("request_duration")

                elif response_command == "duration":

                    cst.simulation_duration = int(msg_split[1])
                    inflog(f"ns3 Simulation duration is {cst.simulation_duration}")
                    inflog("Requesting NetAnim Node count")
                    tap_sender_control("request_node")

                elif response_command == "node":

                    cst.nb_nodes = int(msg_split[1])
                    inflog(f"ns3 Simulation Node count is {cst.nb_nodes}")
                    inflog("Requesting NetAnim animation file")
                    tap_sender_control("request_animfile")

                elif response_command == "file":

                    anim_file = msg_split[1]
                    inflog(f"ns3 Simulation saved on file {anim_file}")
                    inflog("Initializing carla")
                    init_mock()
                    positions = get_all_position()
                    tap_sender_control(f"create_node {positions}")

                elif response_command == "create_success":

                    sockets = []
                    for num_node in range(1, cst.nb_nodes+1):
                        tap_socket = connect_tap_device(f"tap{num_node}")
                        sockets.append(tap_socket)
                    launch_simulation(sockets, socket_tap0)

                else:
                    errlog(f"Erreur message recue : {message}")

        except UnicodeDecodeError as e:
            errlog(f"Erreur de décodage Unicode : {e}")
            inflog("Le paquet reçu ne peut pas être décodé en UTF-8.")
            stop_simulation()

        except Exception as e:
            errlog(f"Erreur inattendue lors du traitement du paquet : {e}")
            stop_simulation()


def launch_simulation(sockets, control_socket):
    """Lance la simulation."""

    interval = 1

    # Lancer l'écoute périodique des positions dans un thread
    inflog("Launching  periodic_position_sender")
    cst.position_listener_thread = threading.Thread(
        target=periodic_position_sender, args=(interval,))
    cst.position_listener_thread.start()

    inflog("Launching comunication_node")
    cst.comunication_nodes_thread = threading.Thread(
        target=comunication_node, args=(interval,))
    cst.comunication_nodes_thread.start()

    inflog("Launching listen_tap_devices")
    cst.listen_tap_devices_thread = threading.Thread(
        target=listen_tap_devices, args=(sockets,))
    cst.listen_tap_devices_thread.start()

    listen_control_tap(control_socket)


def stop_simulation():
    """Arrete la simulation."""

    inflog("Stopping all threads")
    cst.stop_state = True

    if cst.position_listener_thread is not None:
        cst.position_listener_thread.join()
        inflog("periodic_position_sender is stopped")

    if cst.comunication_nodes_thread is not None:
        cst.comunication_nodes_thread.join()
        inflog("cst.comunication_nodes_thread is stopped")

    if cst.listen_tap_devices_thread is not None:
        cst.listen_tap_devices_thread.join()
        inflog("cst.listen_tap_devices_thread is stopped")

    # Arrete les vehicules dans ns3
    if len(cst.vehicles) > 0:
        inflog("Stoping vehicules in ns3")
        tap_sender_control(f"set_mobility {stop_vehicules()}")

    inflog("Simulation terminée.")
    if cst.number_message_sent != 0:
        pdr = (cst.number_message_received/cst.number_message_sent)*100
    else:
        pdr = 0

    inflog(f"Nombre de paquet envoyés: {cst.number_message_sent}")
    inflog(f"Nombre de paquet recu: {cst.number_message_received}")
    inflog(f"Taux de livraison des paquets: PDR = {pdr}%")
    inflog("Interruption reseau avec NS3")
    rclpy.shutdown()
    sys.exit()

#  Threads


def listen_control_tap(control_socket):
    """Écoute sur le tape de controle et controle les threads du programme."""
    while rclpy.ok():
        try:
            if cst.error_state:
                errlog("Erreur dans l'un des threads")
                stop_simulation()

            packet = control_socket.recv(cst.MTU)

            # Récupérer l'adresse IP de destination du paquet
            dest_ip = packet[30:34]

            # Convertir l'adresse IP de destination en une chaine lisible
            dest_ip_str = ".".join(str(byte) for byte in dest_ip)

            # Vérifiez si le paquet est destiné
            # à votre propre adresse TAP pour l'ignorer

            if check_message(packet) and dest_ip_str != '10.0.0.2':
                inflog("tap0 received a packet from control node")
                message = (packet[42:].decode()).rstrip("\n")
                inflog(f"Received packet (decoded): {message}")
                msg_split = message.split(" ")
                command = msg_split[0]

                if command == "time":

                    simulation_time = int(msg_split[1])
                    inflog(f"ns3 Simulation time is {simulation_time} seconds")

                    if cst.simulation_duration - simulation_time < 5:
                        inflog("Fin de la simulation")
                        stop_simulation()

                else:
                    errlog(f"Erreur message recue : {message}")

        except UnicodeDecodeError as e:
            errlog(f"Erreur de décodage Unicode : {e}")
            inflog("Le paquet reçu ne peut pas être décodé en UTF-8.")
            stop_simulation()

        except Exception as e:
            errlog(f"Erreur inattendue lors du traitement du paquet : {e}")
            stop_simulation()


def listen_tap_devices(tap_sockets):
    """Écoute les tap devices."""
    while rclpy.ok():
        try:
            if cst.error_state or cst.stop_state:
                inflog("Exiting listen_tap_devices")
                sys.exit()

            # Utiliser select pour écouter plusieurs sockets
            readable, _, _ = select.select(tap_sockets, [], [], 1.0)
            for sock in readable:
                data, _ = sock.recvfrom(65535)  # Taille max d'un paquet
                device_name = sock.getsockname()[0]  # Nom du device lié
                if check_message(data):
                    message = (data[42:].decode()).rstrip("\n")
                    splits = message.split(" ")
                    if splits[0] == device_name.replace("tap", ""):
                        # compte uniquement si l'on est la cible

                        cst.number_message_received += 1
                        # A faire: traier ce qu'on recoit sur tap1,2,3,...
                        inflog(f"{device_name} received a packet from tap{splits[1]}")
                        inflog(f"Received packet (decoded): {message}")

        except Exception as e:
            errlog(f"Erreur lors de l'écoute des tap devices: {e}")
            cst.error_state = True


def periodic_position_sender(interval):
    """
    Envoie sur tap0 les informations de controle.

    Envoie les positions et vitesses de tous les véhicule.
    Demande également le temps de la simulation.
    """
    while rclpy.ok():
        if cst.error_state or cst.stop_state:
            inflog("Exiting periodic_position_sender")
            sys.exit()
        try:
            mobilities = get_all_mobility()
            tap_sender_control(f"set_mobility {mobilities}")
            tap_sender_control("request_time")
            time.sleep(interval)
        except Exception as e:
            errlog(
                "Erreur lors de la récupération/"
                + f"envoie périodique des positions : {e}")
            cst.error_state = True


def comunication_node(interval):
    """
    Génère du traffic réseau dans ns3.

    Envoie sur tous les tap la position du véhicule avec une destination
    aléatoire pour que cette information soit transmise par Wave dans NS3
    """
    while rclpy.ok():
        if cst.error_state or cst.stop_state:
            inflog("Exiting comunication_node")
            sys.exit()
        try:

            for node in range(1, cst.nb_nodes+1):
                dest_node = node
                while dest_node == node:
                    dest_node = random.randint(1, cst.nb_nodes)
                position = get_position(cst.vehicles[node-1])
                tap_sender(f"{dest_node} {node} position {position}", node)
                cst.number_message_sent += 1
            time.sleep(interval)
        except Exception as e:
            errlog("Erreur lors de la récupération/"
                   + f"envoie périodique des positions : {e}")
            cst.error_state = True


if __name__ == '__main__':
    main()
