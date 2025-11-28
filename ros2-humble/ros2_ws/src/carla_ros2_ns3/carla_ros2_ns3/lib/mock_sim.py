#!/usr/bin/env python

import random

from carla_ros2_ns3.const import NB_NODE
from carla_ros2_ns3.lib.mock_vehicule import Vehicule

# Variables
vehicles = []


def init_mock():
    """Initialise les mocks."""
    # Créer les véhicules
    for _ in range(NB_NODE):
        vehicle = None
        while vehicle is None:
            x = random.randint(-50, 50)
            y = random.randint(-50, 50)
            z = 0
            vx = 0
            vy = 0
            vz = 0
            vehicle = Vehicule(x, y, z, vx, vy, vz)
        vehicles.append(vehicle)


def get_position(vehicle):
    """Recupère la position d'un vehicule mock."""
    try:
        location = vehicle.get_location()
        location_string = f"{location[0]} {location[1]} {location[2]}"
        return location_string
    except Exception as e:
        raise e


def get_speed(vehicle):
    """Recupère la vitesse d'un vehicule mock."""
    try:
        velocity = vehicle.get_velocity()
        velocity_string = f"{velocity[0]} {velocity[1]} {velocity[2]}"
        return velocity_string
    except Exception as e:
        raise e


def get_all_position():
    """Recupère les positions de tous les vehicules mock."""
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
    """Recupère les vitesses de tous les vehicules mock."""
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
    """Recupère les positions et vitesses de tous les vehicules mock."""
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
