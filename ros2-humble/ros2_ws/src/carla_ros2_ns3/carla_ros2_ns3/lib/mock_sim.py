#!/usr/bin/env python

import random

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.mock_vehicule import Vehicule, Transform, Location, Velocity


def init_mock():
    """Initialise les mocks."""
    # Créer les véhicules
    for _ in range(cst.nb_nodes):
        vehicle = None
        while vehicle is None:
            x = random.randint(-50, 50)
            y = random.randint(-50, 50)
            z = 0
            vx = 0
            vy = 0
            vz = 0
            vehicle = Vehicule(Transform(Location(x, y, z)), Velocity(vx, vy, vz))
        cst.vehicles.append(vehicle)
