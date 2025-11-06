#!/usr/bin/env python

import socket
import carla
import random
import rclpy
from std_msgs.msg import String
import time
import sys
# import os
# import subprocess
import select
import threading
import datetime


number_node = 5  # nombre de nodes et donc de voitures dans la simulation
number_message_sent = 0  # Pour les messages s'envoyant via tap1,2,3,...
number_message_received = 0  # Pour les messages s'envoyant via tap1,2,3,...
vehicles = []
client = carla.Client('localhost', 2000)  # connexion a Carla
node = None


def main():
    global node
    rclpy.init(args=sys.argv)
    try:
        node_name = "carla_ros2_ns3"
        node = rclpy.create_node(node_name)
        socket_tap0 = connect_tap_device("tap0")
        tap_sender_control("hello_ROS2")
        control_node_listener(socket_tap0)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

# Utilitaires


def inflog(msg):
    node.get_logger().info(msg)


def errlog(msg):
    node.get_logger().error(msg)

# Partie Réseau


def check_message(rawdata, hidden):
    # check udp
    if rawdata[23] == 17 and rawdata[30] == 10:

        source_ip = f"{rawdata[26]}.{rawdata[27]}.{rawdata[28]}.{rawdata[29]}"
        dest_ip = f"{rawdata[30]}.{rawdata[31]}.{rawdata[32]}.{rawdata[33]}"
        src_port = rawdata[34]*256 + rawdata[35]
        dest_port = rawdata[36]*256 + rawdata[37]
        checksum = hex(rawdata[40]*256 + rawdata[41]).upper()
        udp_payload = rawdata[42:]  # Payload

        if not hidden:
            print("UDP Message:")
            print(f"  Source: {source_ip}")
            print(f"  Destination: {dest_ip}")
            print(f"  Source port: {src_port}")
            print(f"  Destination port:  {dest_port}")
            print("  Length: " + str(rawdata[38]*256 + rawdata[39] - 8))
            print(f"  Checksum: {checksum}")

            checksum = calculate_udp_checksum(
                source_ip, dest_ip, src_port, dest_port, udp_payload)
            print(f"  Calculated UDP checksum: 0x{checksum:04X}")

        return True

    else:
        return False


def calculate_udp_checksum(
        source_ip, dest_ip, src_port, dest_port, udp_payload):
    # Pour convertir une adresse IP en une liste de mots de 16 bits
    def ip_to_words(ip):
        parts = list(map(int, ip.split(".")))
        return [(parts[0] << 8) + parts[1], (parts[2] << 8) + parts[3]]

    def add_with_carry(a, b):
        result = a + b
        return (result & 0xFFFF) + (result >> 16)

    def calculate_checksum(data_words):
        checksum = 0
        for word in data_words:
            checksum = add_with_carry(checksum, word)
        return ~checksum & 0xFFFF

    pseudo_header = (
        ip_to_words(source_ip) + ip_to_words(dest_ip)
        + [0x0011, len(udp_payload) + 8])

    udp_header = [src_port, dest_port, len(udp_payload) + 8, 0]

    payload_words = [
        ((udp_payload[i] << 8) +
         (udp_payload[i + 1] if i + 1 < len(udp_payload) else 0))
        for i in range(0, len(udp_payload), 2)
    ]

    all_words = pseudo_header + udp_header + payload_words
    return calculate_checksum(all_words)


def connect_tap_device(tap_device):
    """
    Établit une connexion au périphérique TAP avec tentative de
    reconnexion en boucle si elle échoue.
    Retourne le socket connecté.
    """
    while rclpy.ok():
        try:
            inflog(f"Tentative de connexion à {tap_device}...")
            s = socket.socket(
                socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003))
            s.bind((tap_device, 0))
            inflog(f"Connexion réussie à {tap_device}.")
            return s
        except Exception as e:
            errlog(
                "Erreur lors de la connexion au périphérique"
                + f"{tap_device}: {e}")

            inflog("Nouvelle tentative dans 5 secondes...")
            time.sleep(5)  # Attendre 5 secondes avant de réessayer


def tap_sender(message, num_node):
    """
    Permet d'envoyer un message grace a un numéro de noeud
    """
    # Créer un éditeur pour publier les paquets sur un topic spécifique
    topic_name = f'/tap{num_node}_packets'
    pub = node.create_publisher(String, topic_name, 10)

    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_ip = f"10.0.{num_node}.1"
        udp_port = 12000+num_node
        null_term_msg = message + "\0"
        packet = null_term_msg.encode("utf-8")
        udp_socket.sendto(packet, (udp_ip, udp_port))
        udp_socket.close()
        inflog(f"Message envoyé à {udp_ip}:{udp_port} : {message}")
        # Publier le paquet sous forme hexadécimale
        msg = String()
        msg.data = packet.hex()
        pub.publish(msg)

    except Exception as e:
        errlog(f"Erreur lors de l'envoi du message : {e}")


def tap_sender_control(message):
    """
    Permet d'envoyer un message grace a un numéro de noeud
    """
    # Créer un éditeur pour publier les paquets sur un topic spécifique
    topic_name = "/tap0_packets"
    pub = node.create_publisher(String, topic_name, 10)

    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_ip = "10.0.0.2"
        udp_port = 12000
        null_term_msg = message + "\0"
        packet = null_term_msg.encode("utf-8")
        udp_socket.sendto(packet, (udp_ip, udp_port))
        udp_socket.close()
        inflog(f"Message envoyé à {udp_ip}:{udp_port} : {message}")
        # Publier le paquet sous forme hexadécimale
        msg = String()
        msg.data = packet.hex()
        pub.publish(msg)

    except Exception as e:
        errlog(f"Erreur lors de l'envoi du message : {e}")


def listen_tap_devices(tap_sockets):
    """
    Écoute sur plusieurs tap devices simultanément
    (pas le tap de controle: le tap0)
    """
    global number_message_received
    while rclpy.ok():
        try:
            # Utiliser select pour écouter plusieurs sockets
            readable, _, _ = select.select(tap_sockets, [], [], 1.0)
            for sock in readable:
                data, _ = sock.recvfrom(65535)  # Taille max d'un paquet
                device_name = sock.getsockname()[0]  # Nom du device lié
                if check_message(data, True):
                    message = (data[42:].decode()).rstrip("\n")
                    if (message.split(" ")[0]
                       != device_name.replace("tap", "")):
                        # exclu le message si l'envoyeur
                        # est le meme que le recepteur

                        number_message_received += 1
                        # A faire: traier ce qu'on recoit sur tap1,2,3,...
                        inflog(f"Données reçues sur {device_name}")

        except Exception as e:
            errlog(f"Erreur lors de l'écoute des tap devices: {e}")
            stop_simulation()


def control_node_listener(socket_tap0):
    """
    Permet d'ecouter ce que recoit tap0, noeud de control
    """
    while rclpy.ok():
        try:

            MTU = 15000  # Maximum Transmission Unit pour Ethernet frame
            packet = socket_tap0.recv(MTU)
            inflog("tap0 received a packet")

            # Récupérer l'adresse IP de destination du paquet
            dest_ip = packet[30:34]

            # Convertir l'adresse IP de destination en une chaine lisible
            dest_ip_str = ".".join(str(byte) for byte in dest_ip)

            # Vérifiez si le paquet est destiné
            # à votre propre adresse TAP pour l'ignorer
            if dest_ip_str == '10.0.0.2':
                inflog(
                    f"Message destiné à {dest_ip_str} (envoi propre) ignoré.")

            elif check_message(packet, False):
                message = (packet[42:].decode()).rstrip("\n")
                inflog(f"Received packet (decoded): {message}")

                if (message == "hello_NS3"):

                    now = datetime.datetime.now()
                    time_str = now.strftime("%Y%m%d_%H%M")
                    inflog("Setting time")
                    tap_sender_control(f"time {time_str}")

                elif (message == "time_success"):

                    inflog("Initializing carla")
                    init_carla()
                    positions = get_all_position()
                    tap_sender_control(f"create_node {positions}")

                elif (message == "create_success"):

                    sockets = []
                    for num_node in range(1, number_node+1):
                        socket = connect_tap_device(f"tap{num_node}")
                        sockets.append(socket)
                    launch_simulation(sockets)

                else:
                    errlog(f"Erreur message recue : {message}")

        except UnicodeDecodeError as e:
            errlog(f"Erreur de décodage Unicode : {e}")
            inflog("Le paquet reçu ne peut pas être décodé en UTF-8.")
            stop_simulation()

        except Exception as e:
            errlog(f"Erreur inattendue lors du traitement du paquet : {e}")
            stop_simulation()

# Partie CARLA


def init_carla():
    """
    Initialise la connexion à Carla, configure le monde et spawn un véhicule.
    """

    global vehicles

    client.set_timeout(20.0)
    # client.load_world("Town01")
    # Pour changer la carte
    world = client.get_world()
    settings = world.get_settings()
    # settings.no_rendering_mode = True
    # Pour desactiver l'utilisation du gpu
    settings.synchronous_mode = False
    world.apply_settings(settings)

    weather = world.get_weather()
    # weather.sun_altitude_angle = 45.0
    # angle du soleil => pour changer l'heure
    world.set_weather(weather)

    # Générer des waypoints pour s'assurer que la carte est prête
    map = world.get_map()
    waypoints = map.generate_waypoints(2.0)
    if not waypoints:
        errlog("Aucun waypoint trouvé sur la carte.")
    else:
        inflog(f"{len(waypoints)} waypoints générés.")

    # Créer les véhicules
    for _ in range(number_node):
        vehicle = None
        while vehicle is None:
            vehicle = spawn_vehicle(world)
        vehicles.append(vehicle)
    return world


def spawn_vehicle(world):
    """
    Crée et initialise un véhicule dans le simulateur Carla.
    """
    # Obtenir la bibliothèque de blueprints
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))

    # Choisir un point d'apparition aléatoire
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        errlog("Aucun point d'apparition disponible.")
        return None
    spawn_point = random.choice(spawn_points)

    # Essayer de créer le véhicule
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    return vehicle


def launch_simulation(sockets):
    """
    Met les voitures en mouvement et lance la collecte de données sur Carla
    (position,vitesse) pour les envoyer par la suite
    """
    traffic_manager = client.get_trafficmanager(8001)
    traffic_manager.set_synchronous_mode(False)
    # Désactiver le mode synchrone du Traffic Manager
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    # Distance minimale

    for vehicle in vehicles:
        vehicle.set_autopilot(True, 8001)

    # Lancer l'écoute périodique des positions dans un thread
    inflog("Launching  periodic_position_sender")
    position_listener_thread = threading.Thread(
        target=periodic_position_sender, args=(1,))
    position_listener_thread.start()

    inflog("Launching comunication_node")
    comunication_nodes_thread = threading.Thread(
        target=comunication_node, args=(1,))
    comunication_nodes_thread.start()

    listen_tap_devices(sockets)


def stop_simulation():

    # Ce code fonctionnait auparavant mais mene a des bugs desormais,
    # il servait a enlever les voitures a la fin de la simulation
    # afin de ne pas devoir rellancer Carla a chaque fois
    # -> a corriger pour eviter cela
    # Détruire les véhicules pour nettoyer la simulation
    # for vehicle in vehicles:
    #    vehicle.destroy()
    #    rospy.loginfo(f"Véhicule {vehicle.id} détruit.")

    inflog("Simulation terminée.")
    if number_message_sent != 0:
        PDR = (number_message_received/number_message_sent)*100
        PDR_division = PDR/(number_node-1)
    else:
        PDR = 0
        PDR_division = 0

    inflog(f"Nombre de paquet envoyés: {number_message_sent}")
    inflog(f"Nombre de paquet recu: {number_message_received}")
    inflog(f"Taux de livraison des paquets: PDR = {PDR}%")
    inflog("Taux de livraison des paquets en prenant en compte"
           + f"un broadcast: PDR = {PDR_division}%")
    inflog("Interruption reseau avec NS3")
    rclpy.shutdown()


def get_position(vehicle):
    transform = vehicle.get_transform()
    location = transform.location
    location_string = f"{location.x} {location.y} {location.z}"
    return location_string


def get_speed(vehicle):
    velocity = vehicle.get_velocity()
    velocity_string = f"{velocity.x} {velocity.y} {velocity.z}"
    return velocity_string


def get_all_position():
    output = " "
    index_vehicle = 1
    for vehicle in vehicles:
        output += f" {index_vehicle} {get_position(vehicle)}"
        if index_vehicle < number_node:
            output += " "
            index_vehicle += 1

    return output


"""
def get_all_speed():
    output = ""
    index_vehicle = 1
    for vehicle in vehicles:
        output += f" {index_vehicle} {get_speed(vehicle)}"
        index_vehicle += 1
    return output
"""


def get_all_mobility():
    output = " "
    index_vehicle = 1
    for vehicle in vehicles:
        output += (f"{index_vehicle} {get_position(vehicle)} "
                   + f"{get_speed(vehicle)}")
        if index_vehicle < number_node:
            output += " "
            index_vehicle += 1

    return output

#  Threads


def periodic_position_sender(interval):
    """
    Envoie sur tap0 les positions et vitesses de tout les véhicule.
    """
    while rclpy.ok():
        try:
            mobilities = get_all_mobility()
            tap_sender_control(f"set_mobility {mobilities}")
            time.sleep(interval)
        except Exception as e:
            errlog(
                "Erreur lors de la récupération/"
                + f"envoie périodique des positions : {e}")
            stop_simulation()


def comunication_node(interval):
    """
    Envoie sur un tap (ici pris aléatoirement pour les tests) la position
    d'un véhicule pour que cette information soit transmise par Wave dans NS3
    """
    global number_message_sent
    while rclpy.ok():
        try:
            num_node = random.randint(1, number_node)
            position = get_position(vehicles[num_node-1])
            tap_sender(f"{num_node} position {position}", num_node)
            number_message_sent += 1
            time.sleep(interval)
        except Exception as e:
            errlog("Erreur lors de la récupération/"
                   + f"envoie périodique des positions : {e}")
            stop_simulation()


if __name__ == '__main__':
    main()
