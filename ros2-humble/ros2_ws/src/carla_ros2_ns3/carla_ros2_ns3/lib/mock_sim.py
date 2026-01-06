#!/usr/bin/env python

import random

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.mock_vehicule import ConstantPositionVehicule, ConstantVelocityVehicule


def init_mock():
    """Initialise les mocks."""
    # Créer les véhicules
    for i in range(cst.nb_nodes):
        if i == 0:
            x = 0.0
            y = 0.0
            z = 0.0
            vehicle = ConstantPositionVehicule(i, x, y, z)
        elif i == 1:
            x = 10.0
            y = 0.0
            z = 0.0
            vx = 1.0
            vy = 0.0
            vz = 0.0
            vehicle = ConstantVelocityVehicule(i, x, y, z, vx, vy, vz)
        else:
            x = random.uniform(-50.0, 50.0)
            y = random.uniform(-50.0, 50.0)
            z = 0.0
            vehicle = ConstantPositionVehicule(i, x, y, z)
        cst.vehicles.append(vehicle)
