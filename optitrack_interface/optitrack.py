# WIFI CONFIG
# SSID: OptiTrack
# PSW: 60A84A244BECD

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from rclpy import qos
import socket
import struct
import numpy
from threading import Thread
from time import time
from math import atan2, asin
from optitrack_interface.nat_net_client import NatNetClient

TRACKED_ROBOT_ID = 20 # IMPORTANT: must match the streaming ID of the optitrack program


class Optitrack(Node):
  def __init__(self):
    super().__init__('optitrack_node')
    self.publisher_ = self.create_publisher(PoseStamped, '/optitrack/pose', qos.qos_profile_sensor_data)

    self.calls = 0

    self.timer = self.create_timer(1.0, self.timer_callback)

    self.get_logger().info('Optitrack node running - version 2.1')

    streamingClient = NatNetClient(ver=(3, 2, 0, 0), quiet=True)
    streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
    streamingClient.run()

  def timer_callback(self):
    print("Hz: "+str(self.calls))
    self.calls = 0

  def receiveRigidBodyFrame(self, id, position, rotation):
    if (id==TRACKED_ROBOT_ID):
      msg = PoseStamped()
      # retrieve as originally published by the optitrack
      msg.header.frame_id = "tag"
      msg.header.stamp = self.get_clock().now().to_msg()

      msg.pose.position.x = position[0]
      msg.pose.position.y = position[1]
      msg.pose.position.z = position[2]

      msg.pose.orientation.w = rotation[3] # q_w
      msg.pose.orientation.x = rotation[2] # q_x
      msg.pose.orientation.y = rotation[1] # q_y
      msg.pose.orientation.z = rotation[0] # q_z

      self.publisher_.publish(msg)
      self.calls = self.calls + 1
    else:
      self.get_logger().info('Message with different tracker ID' + str(id))


            

def main(args=None):
  rclpy.init(args=args)

  optitrack = Optitrack()

  rclpy.spin(optitrack)

  optitrack.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
