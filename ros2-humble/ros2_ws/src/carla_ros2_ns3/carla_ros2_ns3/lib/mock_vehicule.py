#!/usr/bin/env python

class Vehicule:
    """A basic mock for carla Vehicule."""

    def __init__(self, x, y, z, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def get_location(self):
        """Return a mock vehicule location."""
        return self.x, self.y, self.z

    def get_velocity(self):
        """Return a mock vehicule velocity."""
        return self.vx, self.vy, self.vz
