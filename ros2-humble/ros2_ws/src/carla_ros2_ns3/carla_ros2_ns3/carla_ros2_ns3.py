#!/usr/bin/env python

import socket
import random
import time
import sys
import select
import threading

import carla
import rclpy
from std_msgs.msg import String

# Constantes
NB_NODE = 5  # nombre de nodes et donc de voitures dans la simulation
MTU = 15000  # Maximum Transmission Unit pour Ethernet frame
# Variables de simulation
number_message_sent = 0  # Pour les messages s'envoyant via tap1,2,3,...
number_message_received = 0  # Pour les messages s'envoyant via tap1,2,3,...
vehicles = []
client = carla.Client('localhost', 2000)  # connexion a Carla
netAnim_file = ""
simulation_duration = 0
# Noeud principal
node = None
# Thread et essentiels pour la sychronisation
position_listener_thread = None
comunication_nodes_thread = None
listen_tap_devices_thread = None
stop_state = False
error_state = False


def main():
    """Initialise le noeud principal et lance le programme."""
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
    """Logger d'info pour le noeud principal."""
    node.get_logger().info(msg)


def errlog(msg):
    """Logger d'erreur pour le noeud principal."""
    node.get_logger().error(msg)

# Partie Réseau


def check_message(rawdata):
    """Verify que le message est UDP et la destination est dans 10.0.0.0."""
    # check udp
    if rawdata[23] == 17 and rawdata[30] == 10:
        return True
    return False


def print_udp(rawdata):
    """Print an UDP packet."""
    source_ip = f"{rawdata[26]}.{rawdata[27]}.{rawdata[28]}.{rawdata[29]}"
    dest_ip = f"{rawdata[30]}.{rawdata[31]}.{rawdata[32]}.{rawdata[33]}"
    src_port = rawdata[34]*256 + rawdata[35]
    dest_port = rawdata[36]*256 + rawdata[37]
    checksum = hex(rawdata[40]*256 + rawdata[41]).upper()
    udp_payload = rawdata[42:]  # Payload

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


def calculate_udp_checksum(
        source_ip, dest_ip, src_port, dest_port, udp_payload):
    """Calculate la somme de controle d'un paquet UDP."""
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
    Établit une connexion au périphérique TAP.

    Reconnexion en boucle si elle échoue.
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
    return None


def tap_sender(message, num_node):
    """Permet d'envoyer un message grace a un numéro de noeud."""
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
    """Permet d'envoyer un message au noeud de controle."""
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


def control_node_listener(socket_tap0):
    """Permet d'ecouter ce que recoit tap0, noeud de controle."""
    global netAnim_file
    global simulation_duration

    while rclpy.ok():
        try:

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

            elif check_message(packet):
                print_udp(packet)
                message = (packet[42:].decode()).rstrip("\n")
                inflog(f"Received packet (decoded): {message}")
                msg_split = message.split(" ")
                response_command = msg_split[0]

                if response_command == "hello_NS3":

                    inflog("Requesting simulation duration")
                    tap_sender_control("request_duration")

                elif response_command == "duration":

                    simulation_duration = int(msg_split[1])
                    inflog(f"ns3 Simulation duration is {simulation_duration}")
                    inflog("Requesting NetAnim animation file")
                    tap_sender_control("request_animfile")

                elif response_command == "file":

                    netAnim_file = msg_split[1]
                    inflog(f"ns3 Simulation saved on file {netAnim_file}")
                    inflog("Initializing carla")
                    init_carla()
                    positions = get_all_position()
                    tap_sender_control(f"create_node {positions}")

                elif response_command == "create_success":

                    sockets = []
                    for num_node in range(1, NB_NODE+1):
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

# Partie CARLA


def init_carla():
    """Initialise la connexion à Carla."""
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
    for _ in range(NB_NODE):
        vehicle = None
        while vehicle is None:
            vehicle = spawn_vehicle(world)
        vehicles.append(vehicle)
    return world


def spawn_vehicle(world):
    """Crée et initialise un véhicule dans le simulateur Carla."""
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


def launch_simulation(sockets, control_socket):
    """Lance la simulation."""
    global position_listener_thread
    global comunication_nodes_thread
    global listen_tap_devices_thread

    traffic_manager = client.get_trafficmanager(8001)
    traffic_manager.set_synchronous_mode(False)
    # Désactiver le mode synchrone du Traffic Manager
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    # Distance minimale

    for vehicle in vehicles:
        vehicle.set_autopilot(True, 8001)

    interval = 1

    # Lancer l'écoute périodique des positions dans un thread
    inflog("Launching  periodic_position_sender")
    position_listener_thread = threading.Thread(
        target=periodic_position_sender, args=(interval,))
    position_listener_thread.start()

    inflog("Launching comunication_node")
    comunication_nodes_thread = threading.Thread(
        target=comunication_node, args=(interval,))
    comunication_nodes_thread.start()

    inflog("Launching listen_tap_devices")
    listen_tap_devices_thread = threading.Thread(
        target=listen_tap_devices, args=(sockets,))
    listen_tap_devices_thread.start()

    listen_control_tap(control_socket)


def stop_simulation():
    """Arrete la simulation."""
    global stop_state

    inflog("Stopping all threads")
    stop_state = True

    if position_listener_thread is not None:
        position_listener_thread.join()
        inflog("periodic_position_sender is stopped")

    if comunication_nodes_thread is not None:
        comunication_nodes_thread.join()
        inflog("comunication_nodes_thread is stopped")

    if listen_tap_devices_thread is not None:
        listen_tap_devices_thread.join()
        inflog("listen_tap_devices_thread is stopped")

    # Arrete les vehicules dans ns3
    if len(vehicles) > 0:
        inflog("Stoping vehicules in ns3")
        tap_sender_control(f"set_mobility {stop_vehicules()}")

    # Détruire les véhicules pour nettoyer la simulation
    for vehicle in vehicles:
        if vehicle.is_alive:
            vid = vehicle.id
            vehicle.set_autopilot(False, 8001)
            vehicle.destroy()
            inflog(f"Véhicule {vid} détruit.")

    inflog("Simulation terminée.")
    if number_message_sent != 0:
        pdr = (number_message_received/number_message_sent)*100
        pdr_division = pdr/(NB_NODE-1)
    else:
        pdr = 0
        pdr_division = 0

    inflog(f"Nombre de paquet envoyés: {number_message_sent}")
    inflog(f"Nombre de paquet recu: {number_message_received}")
    inflog(f"Taux de livraison des paquets: PDR = {pdr}%")
    inflog("Taux de livraison des paquets en prenant en compte"
           + f"un broadcast: PDR = {pdr_division}%")
    inflog("Interruption reseau avec NS3")
    rclpy.shutdown()
    sys.exit()


def get_position(vehicle):
    """Recupère la position d'un vehicule carla."""
    try:
        transform = vehicle.get_transform()
        location = transform.location
        location_string = f"{location.x} {location.y} {location.z}"
        return location_string
    except Exception as e:
        raise e


def get_speed(vehicle):
    """Recupère la vitesse d'un vehicule carla."""
    try:
        velocity = vehicle.get_velocity()
        velocity_string = f"{velocity.x} {velocity.y} {velocity.z}"
        return velocity_string
    except Exception as e:
        raise e


def get_all_position():
    """Recupère les positions de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in vehicles:
            output += f" {index_vehicle} {get_position(vehicle)}"
            if index_vehicle < NB_NODE:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        raise e


def get_all_speed():
    """Recupère les vitesses de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in vehicles:
            output += f" {index_vehicle} {get_speed(vehicle)}"
            if index_vehicle < NB_NODE:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        raise e


def get_all_mobility():
    """Recupère les positions et vitesses de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in vehicles:
            output += (f"{index_vehicle} {get_position(vehicle)} "
                       + f"{get_speed(vehicle)}")
            if index_vehicle < NB_NODE:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        raise e


def stop_vehicules():
    """Recupère les positions et met leur vitesse à 0 pour les vehicules."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in vehicles:
            output += (f"{index_vehicle} {get_position(vehicle)} 0.0 0.0 0.0")
            if index_vehicle < NB_NODE:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        raise e

#  Threads


def listen_control_tap(control_socket):
    """Écoute sur le tape de controle et controle les threads du programme."""
    while rclpy.ok():
        try:
            if error_state:
                errlog("Erreur dans l'un des threads")
                stop_simulation()

            packet = control_socket.recv(MTU)
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

            elif check_message(packet):
                message = (packet[42:].decode()).rstrip("\n")
                inflog(f"Received packet (decoded): {message}")
                msg_split = message.split(" ")
                command = msg_split[0]

                if command == "time":

                    simulation_time = int(msg_split[1])
                    inflog(f"ns3 Simulation time is {simulation_time} seconds")

                    if simulation_duration - simulation_time < 5:
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
    global number_message_received
    global error_state
    while rclpy.ok():
        try:
            if error_state or stop_state:
                inflog("Exiting listen_tap_devices")
                sys.exit()

            # Utiliser select pour écouter plusieurs sockets
            readable, _, _ = select.select(tap_sockets, [], [], 1.0)
            for sock in readable:
                data, _ = sock.recvfrom(65535)  # Taille max d'un paquet
                device_name = sock.getsockname()[0]  # Nom du device lié
                if check_message(data):
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
            error_state = True


def periodic_position_sender(interval):
    """
    Envoie sur tap0 les informations de controle.

    Envoie les positions et vitesses de tous les véhicule.
    Demande également le temps de la simulation.
    """
    global error_state
    while rclpy.ok():
        if error_state or stop_state:
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
            error_state = True


def comunication_node(interval):
    """
    Génère du traffic réseau dans ns3.

    Envoie sur un tap (pris aléatoirement pour les tests) la position
    d'un véhicule pour que cette information soit transmise par Wave dans NS3
    """
    global error_state
    global number_message_sent
    while rclpy.ok():
        if error_state or stop_state:
            inflog("Exiting comunication_node")
            sys.exit()
        try:
            num_node = random.randint(1, NB_NODE)
            position = get_position(vehicles[num_node-1])
            tap_sender(f"{num_node} position {position}", num_node)
            number_message_sent += 1
            time.sleep(interval)
        except Exception as e:
            errlog("Erreur lors de la récupération/"
                   + f"envoie périodique des positions : {e}")
            error_state = True


if __name__ == '__main__':
    main()
