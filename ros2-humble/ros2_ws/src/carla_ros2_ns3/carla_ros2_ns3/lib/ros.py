#!/usr/bin/env python

from std_msgs.msg import String

import rclpy

# Noeud principal
node = None


# Utilitaires

def create_node(node_name):
    """Wrap the node creation."""
    global node
    node = rclpy.create_node(node_name)


def destroy_node():
    """Wrap the node destruction."""
    node.destroy_node()


def create_pub(topic_name):
    """Wrap node.create_publisher(String, topic_name, 10)."""
    return node.create_publisher(String, topic_name, 10)


def inflog(msg):
    """Logger d'info pour le noeud principal."""
    node.get_logger().info(msg)


def errlog(msg):
    """Logger d'erreur pour le noeud principal."""
    node.get_logger().error(msg)
