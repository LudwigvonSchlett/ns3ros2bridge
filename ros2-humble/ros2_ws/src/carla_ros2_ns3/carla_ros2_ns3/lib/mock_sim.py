#!/usr/bin/env python

import random

import carla_ros2_ns3.const as cst
from carla_ros2_ns3.lib.mock_vehicule import ConstantPositionVehicule, ConstantVelocityVehicule
from carla_ros2_ns3.lib.ros import inflog, errlog


def init_mock():
    """Initialise les mocks."""
    if cst.mock_type == "static":
        inflog("Static scenario")
        static_scenario()
    elif cst.mock_type == "mobile":
        inflog("Mobile scenario")
        mobile_scenario()
    elif cst.mock_type == "range_test" and cst.nb_nodes == 2:
        inflog("Range_test scenario")
        range_test_scenario()
    else:
        errlog("No scenario selected, defaulting to static")
        static_scenario()


def static_scenario():
    """Scenario avec des mocks immobiles."""
    # Créer les véhicules
    for i in range(1, cst.nb_nodes + 1):
        x = random.uniform(-50.0, 50.0)
        y = random.uniform(-50.0, 50.0)
        z = 0.0
        vehicle = ConstantPositionVehicule(i, x, y, z)
        cst.vehicles.append(vehicle)


def mobile_scenario():
    """Scenario avec des mocks mobiles."""
    # Créer les véhicules
    for i in range(1, cst.nb_nodes + 1):
        x = 0.0
        y = 0.0
        z = 0.0
        vx = random.uniform(-5.0, 5.0)
        vy = random.uniform(-5.0, 5.0)
        vz = 0.0
        vehicle = ConstantVelocityVehicule(i, x, y, z, vx, vy, vz)
        cst.vehicles.append(vehicle)


def range_test_scenario():
    """Scenario pour comparer des modèles de propagation."""
    # Créer les véhicules
    x = 0.0
    y = 0.0
    z = 0.0
    vehicle_1 = ConstantPositionVehicule(1, x, y, z)
    cst.vehicles.append(vehicle_1)
    x = 10.0
    y = 0.0
    z = 0.0
    vx = 1.0
    vy = 0.0
    vz = 0.0
    vehicle_2 = ConstantVelocityVehicule(2, x, y, z, vx, vy, vz)
    cst.vehicles.append(vehicle_2)
