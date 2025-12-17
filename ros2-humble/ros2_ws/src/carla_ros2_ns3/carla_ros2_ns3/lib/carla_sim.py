#!/usr/bin/env python

import random

import carla

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.ros import (
    inflog,
    errlog
)

# Partie CARLA

# Variables
if cst.MODE == "gpu":
    host = "localhost"
    no_rendering = False
elif cst.MODE == "cpu":
    host = "localhost"
    no_rendering = True
elif cst.MODE == "vm":
    host = "192.168.56.1"
    no_rendering = True
else:
    host = "localhost"
    no_rendering = False

client = carla.Client(host, 2000)  # connexion a Carla


def init_carla():
    """Initialise la connexion à Carla."""

    client = carla.Client(host, 2000)  # connexion a Carla
    client.set_timeout(20.0)
    client.load_world("Town03")
    # Pour changer la carte
    world = client.get_world()
    settings = world.get_settings()
    settings.no_rendering_mode = no_rendering
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
        
    blueprints = world.get_blueprint_library().filter('vehicle.audi.tt')
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)
    
    if cst.nb_nodes < number_of_spawn_points:
                random.shuffle(spawn_points)
    elif cst.nb_nodes > number_of_spawn_points:
        errlog(f"{cst.nb_nodes} noeuds mais {number_of_spawn_points} spawns")
        cst.nb_nodes = number_of_spawn_points

    # Créer les véhicules
    for _ in range(cst.nb_nodes):
        vehicle = None
        while vehicle is None:
            vehicle = spawn_vehicle(world, blueprints, spawn_points)
        cst.vehicles.append(vehicle)
    return world


def spawn_vehicle(world, blueprints, spawn_points):
    """Crée et initialise un véhicule dans le simulateur Carla."""
    # Obtenir la bibliothèque de blueprints
    blueprint = random.choice(blueprints)
    
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)

    # Choisir un point d'apparition aléatoire
    if not spawn_points:
        errlog("Aucun point d'apparition disponible.")
        return None
    spawn_point = random.choice(spawn_points)

    # Essayer de créer le véhicule
    vehicle = world.try_spawn_actor(blueprint, spawn_point)

    return vehicle


def get_position(vehicle):
    """Recupère la position d'un vehicule carla."""
    try:
        transform = vehicle.get_transform()
        location = transform.location
        location_string = f"{location.x} {location.y} {location.z}"
        return location_string
    except Exception as e:
        print(e)
        errlog("Location will be wrong")
        location_string = "0 0 0"
        return location_string


def get_speed(vehicle):
    """Recupère la vitesse d'un vehicule carla."""
    try:
        velocity = vehicle.get_velocity()
        velocity_string = f"{velocity.x} {velocity.y} {velocity.z}"
        return velocity_string
    except Exception as e:
        print(e)
        errlog("Velocity will be wrong")
        velocity_string = "0 0 0"
        return velocity_string


def get_all_position():
    """Recupère les positions de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in cst.vehicles:
            output += f" {index_vehicle} {get_position(vehicle)}"
            if index_vehicle < cst.nb_nodes:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        print(e)


def get_all_speed():
    """Recupère les vitesses de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in cst.vehicles:
            output += f" {index_vehicle} {get_speed(vehicle)}"
            if index_vehicle < cst.nb_nodes:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        print(e)


def get_all_mobility():
    """Recupère les positions et vitesses de tous les vehicules carla."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in cst.vehicles:
            output += (f"{index_vehicle} {get_position(vehicle)} "
                       + f"{get_speed(vehicle)}")
            if index_vehicle < cst.nb_nodes:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        print(e)


def stop_vehicules():
    """Recupère les positions et met leur vitesse à 0 pour les vehicules."""
    try:
        output = " "
        index_vehicle = 1
        for vehicle in cst.vehicles:
            output += (f"{index_vehicle} {get_position(vehicle)} 0.0 0.0 0.0")
            if index_vehicle < cst.nb_nodes:
                output += " "
                index_vehicle += 1
        return output
    except Exception as e:
        print(e)
