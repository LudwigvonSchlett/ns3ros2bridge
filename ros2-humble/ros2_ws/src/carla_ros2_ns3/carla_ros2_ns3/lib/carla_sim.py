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
    HOST = "localhost"
    RENDERING = False
elif cst.MODE == "cpu":
    HOST = "localhost"
    RENDERING = True
elif cst.MODE == "vm":
    HOST = "192.168.56.1"
    RENDERING = True
else:
    HOST = "localhost"
    RENDERING = False

client = carla.Client(HOST, 2000)  # connexion a Carla


def init_carla():
    """Initialise la connexion à Carla."""
    try:
        client.set_timeout(30.0)
        if cst.MODE != "gpu":
            world = client.load_world_if_different('Town01_Opt', False, carla.MapLayer.NONE)
            if world is None:
                world = client.get_world()
        else:
            world = client.load_world_if_different('Town01', False)  # Pour changer la carte
            if world is None:
                world = client.get_world()

        settings = world.get_settings()
        settings.no_rendering_mode = RENDERING  # Pour activer/désactiver l'utilisation du gpu

        settings.synchronous_mode = False
        world.apply_settings(settings)

        # Mise en place de la météo
        weather = world.get_weather()
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.cloudiness = 0.0
        weather.sun_altitude_angle = 90.0  # angle du soleil => pour changer l'heure
        world.set_weather(weather)

        map = world.get_map()
        # Générer des waypoints pour s'assurer que la carte est prête
        # waypoints = map.generate_waypoints(4.0)
        # if not waypoints:
        #     errlog("Aucun waypoint trouvé sur la carte.")
        # else:
        #     inflog(f"{len(waypoints)} waypoints générés.")

        blueprints = world.get_blueprint_library().filter('vehicle.audi.tt')
        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = map.get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if cst.nb_nodes < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif cst.nb_nodes > number_of_spawn_points:
            errlog(f"{cst.nb_nodes} noeuds mais {number_of_spawn_points} spawns")
            cst.nb_nodes = number_of_spawn_points

        # Créer les véhicules
        inflog(f"Creating {cst.nb_nodes} vehicules")
        for _ in range(cst.nb_nodes):
            vehicle = None
            while vehicle is None:
                vehicle = spawn_vehicle(world, blueprints, spawn_points)
            cst.vehicles.append(vehicle)

        inflog("Initializing traffic manager")
        traffic_manager = client.get_trafficmanager(8001)
        traffic_manager.set_synchronous_mode(False)
        # Désactiver le mode synchrone du Traffic Manager
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        # Distance minimale

    except Exception as e:
        errlog("Exception initializing carla")
        raise e


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
