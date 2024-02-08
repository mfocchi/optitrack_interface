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
import sys
TRACKED_ROBOT_ID = 20 # IMPORTANT: must match the streaming ID of the optitrack program


class Optitrack(Node):
  def __init__(self, debug=False):
    super().__init__('optitrack_node')
    self.publisher_ = self.create_publisher(PoseStamped, '/optitrack/pose', qos.qos_profile_sensor_data)

    publisher_frequency = 200.0  # seconds
    self.pub_timer = self.create_timer(1/publisher_frequency, self.pub_timer_callback)
    self.actual_pose = PoseStamped()
    self.feasible_pose = PoseStamped()

    self.debug = debug
    self.calls = 0
    if self.debug:
      self.debug_timer = self.create_timer(1.0, self.debug_timer_callback)

    self.get_logger().info('Optitrack node running - version 2.2')

    streamingClient = NatNetClient(ver=(3, 2, 0, 0), quiet=True)
    streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
    streamingClient.run()

  def debug_timer_callback(self):
    print("Hz: "+str(self.calls))
    self.calls = 0

  def pub_timer_callback(self):
    self.publisher_.publish(self.actual_pose)

  def receiveRigidBodyFrame(self, id, position, rotation):
    if (id==TRACKED_ROBOT_ID):
 
      # retrieve as originally published by the optitrack
      self.actual_pose.header.frame_id = "tag"
      self.actual_pose.header.stamp = self.get_clock().now().to_msg()

      self.actual_pose.pose.position.x = position[0]
      self.actual_pose.pose.position.y = position[1]
      self.actual_pose.pose.position.z = position[2]

      self.actual_pose.pose.orientation.w = rotation[3] # q_w
      self.actual_pose.pose.orientation.x = rotation[2] # q_x
      self.actual_pose.pose.orientation.y = rotation[1] # q_y
      self.actual_pose.pose.orientation.z = rotation[0] # q_z

      #flip quaternion if long path 
      old_quat = np.array(self.feasible.pose.pose.orientation.w, self.actual_pose.pose.orientation.x, self.actual_pose.pose.orientation.y, self.actual_pose.pose.orientation.z)
      new_quat = np.array(self.actual_pose.pose.pose.orientation.w, self.actual_pose.pose.orientation.x, self.actual_pose.pose.orientation.y, self.actual_pose.pose.orientation.z)
      if old_quat.dot(new_quat) < 0:
          #flip new quat
           self.actual_pose.pose.orientation.w *= -1;
           self.actual_pose.pose.orientation.x *= -1;
           self.actual_pose.pose.orientation.y *= -1;
           self.actual_pose.pose.orientation.z *= -1;

      #check discontinuities wrt previous
      if  not( abs(self.actual_pose.pose.orientation.w - self.actual_pose.pose.orientation.w)>0.4 or  
           abs(self.actual_pose.pose.orientation.x - self.actual_pose.pose.orientation.x)>0.4 or 
           abs(self.actual_pose.pose.orientation.y - self.actual_pose.pose.orientation.y)>0.4 or
           abs(self.actual_pose.pose.orientation.z - self.actual_pose.pose.orientation.z)>0.4):
           # update the value if there are no discontiuities
           self.feasible_pose = self.actual_pose
          

      self.calls = self.calls + 1

    else:
      self.get_logger().info('Message with different tracker ID' + str(id))


            

def main(args=None):
  rclpy.init(args=args)
  args = sys.argv[1:]
  debug = False
  if (len(args) == 1):
    if args[0] == '--debug':
      debug = True
  optitrack = Optitrack(debug=debug)

  rclpy.spin(optitrack)

  optitrack.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
