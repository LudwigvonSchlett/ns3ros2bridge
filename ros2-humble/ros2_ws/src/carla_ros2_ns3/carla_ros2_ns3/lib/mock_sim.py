#!/usr/bin/env python

import random

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.mock_vehicule import ConstantPositionVehicule


def init_mock():
    """Initialise les mocks."""
    # Créer les véhicules
    for _ in range(cst.nb_nodes):
        vehicle = None
        while vehicle is None:
            x = random.randint(-50, 50)
            y = random.randint(-50, 50)
            z = 0
            vehicle = ConstantPositionVehicule(x, y, z)
        cst.vehicles.append(vehicle)
