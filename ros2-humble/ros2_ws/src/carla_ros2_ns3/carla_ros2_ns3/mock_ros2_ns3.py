#!/usr/bin/env python


import carla_ros2_ns3.const as cst
import carla_ros2_ns3.carla_ros2_ns3 as base


def main():
    """Initialise le noeud principal et lance le programme."""
    cst.carla_sim = "mock"
    base.main()


if __name__ == '__main__':
    main()
