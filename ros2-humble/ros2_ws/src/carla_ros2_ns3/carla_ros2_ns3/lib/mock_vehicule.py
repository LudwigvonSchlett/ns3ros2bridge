#!/usr/bin/env python

import sys
import threading
import time

import carla_ros2_ns3.const as cst


class Vehicule:
    """A basic mock for carla Vehicule."""

    def __init__(self, node_id, transform, velocity):
        self.id = node_id
        self.transform = transform
        self.velocity = velocity
        self.is_alive = True
        self.stop = True
        self.autopilot_thread = None

    def get_transform(self):
        """Return a mock vehicule transform."""
        return self.transform

    def get_velocity(self):
        """Return a mock vehicule velocity."""
        return self.velocity

    def move(self, mult=1):
        """Move the vehicule."""
        vx = self.velocity.x * mult
        vy = self.velocity.y * mult
        vz = self.velocity.z * mult
        self.transform.move(vx, vy, vz)

    def autopilot(self, interval):
        """Thread for autopilot."""
        while self.stop is False:
            self.move(interval)
            time.sleep(interval)
        sys.exit()

    def destroy(self):
        """Mock vehicule destruction."""
        self.is_alive = False

    def set_autopilot(self, status, port):
        """Set up the autopilot."""
        if status:
            self.stop = False
            self.autopilot_thread = threading.Thread(
                target=self.autopilot, args=(cst.interval,))
            self.autopilot_thread.start()
        else:
            self.stop = True
            self.autopilot_thread.join()


class Transform:
    """A basic mock for carla Transform."""

    def __init__(self, location):
        self.location = location

    def move(self, vx, vy, vz):
        """Set transform location."""
        self.location.x = self.location.x + vx
        self.location.y = self.location.y + vy
        self.location.z = self.location.z + vz


class Location:
    """A basic mock for carla Location."""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Velocity:
    """A basic mock for carla Velocity."""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class ConstantPositionVehicule(Vehicule):
    """A basic mock for a static carla Vehicule."""

    def __init__(self, node_id, x, y, z):
        transform = Transform(Location(x, y, z))
        velocity = Velocity(0, 0, 0)
        Vehicule.__init__(self, node_id, transform, velocity)


class ConstantVelocityVehicule(Vehicule):
    """A basic mock for a carla Vehicule with constant speed."""

    def __init__(self, node_id, x, y, z, vx, vy, vz):
        transform = Transform(Location(x, y, z))
        velocity = Velocity(vx, vy, vz)
        Vehicule.__init__(self, node_id, transform, velocity)
