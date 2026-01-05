#!/usr/bin/env python

class Vehicule:
    """A basic mock for carla Vehicule."""

    def __init__(self, transform, velocity):
        self.transform = transform
        self.velocity = velocity

    def get_transform(self):
        """Return a mock vehicule transform."""
        return self.transform

    def get_velocity(self):
        """Return a mock vehicule velocity."""
        return self.velocity


class Transform:
    """A basic mock for carla Transform."""

    def __init__(self, location):
        self.location = location


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

    def __init__(self, x, y, z):
        transform = Transform(Location(x, y, z))
        velocity = Velocity(0, 0, 0)
        Vehicule.__init__(self, transform, velocity)
