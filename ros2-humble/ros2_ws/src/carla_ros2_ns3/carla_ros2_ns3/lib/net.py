#!/usr/bin/env python

import socket
import time
import struct
import rclpy

from std_msgs.msg import String

from carla_ros2_ns3.lib.ros import (
    inflog,
    errlog,
    create_pub
)

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


def tap_sender(packet, num_node):
    """Permet d'envoyer des bytes grace a un numéro de noeud."""
    # Créer un éditeur pour publier les paquets sur un topic spécifique
    topic_name = f'/tap{num_node}_packets'
    pub = create_pub(topic_name)

    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_ip = f"10.0.{num_node}.1"
        udp_port = 12000+num_node
        udp_socket.sendto(packet, (udp_ip, udp_port))
        udp_socket.close()
        inflog(f"Message envoyé à {udp_ip}:{udp_port} : {packet.hex()}")
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
    pub = create_pub(topic_name)

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


def get_source_tlv(node):
    """Génère le tlv de source pour un noeud."""
    value = struct.pack('=B', node)
    return struct.pack('=BB', 1, len(value)) + value


def get_destination_tlv(node):
    """Génère le tlv de source pour un noeud."""
    value = struct.pack('=B', node)
    return struct.pack('=BB', 2, len(value)) + value


def get_position_tlv(node, vehicle):
    """Recupère la position d'un vehicule carla et génère le tlv."""
    try:
        transform = vehicle.get_transform()
        location = transform.location
        value = struct.pack('=Bxxxfff', node, location.x, location.y, location.z)
        return struct.pack('=BB', 3, len(value)) + value
    except Exception as e:
        print(e)
        errlog("Location will be wrong")
        value = struct.pack('=fffBxxx', node, 0.0, 0.0, 0.0)
        return struct.pack('=BB', 3, len(value)) + value


def get_speed_tlv(node, vehicle):
    """Recupère la vitesse d'un vehicule carla et génère le tlv."""
    try:
        velocity = vehicle.get_velocity()
        value = struct.pack('=Bxxxfff', node, velocity.x, velocity.y, velocity.z)
        return struct.pack('=BB', 4, len(value)) + value
    except Exception as e:
        print(e)
        errlog("Velocity will be wrong")
        value = struct.pack('=Bxxxfff', node, 0.0, 0.0, 0.0)
        return struct.pack('=BB', 4, len(value)) + value


def parse_tlv(message_tlv):
    """Extrait d'un message tlv ses composantes."""
    size = len(message_tlv)
    parse = 0

    src = -1
    dst = -1
    pos = []
    vel = []

    while parse < size:
        tlv_type = message_tlv[parse]
        parse += 1
        length = message_tlv[parse]
        parse += 1

        if tlv_type == 1 and length == 1:
            src = message_tlv[parse]
        elif tlv_type == 2 and length == 1:
            dst = message_tlv[parse]
        elif tlv_type == 3 and length == 16:
            pos.append(struct.unpack("=Bxxxfff", message_tlv[parse:parse+length]))
        elif tlv_type == 4 and length == 16:
            vel.append(struct.unpack("=Bxxxfff", message_tlv[parse:parse+length]))

        parse += length

    return src, dst, pos, vel
