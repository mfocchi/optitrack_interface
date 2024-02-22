# WIFI CONFIG
# SSID: OptiTrack
# PSW: 60A84A244BECD

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
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
import numpy as np
from copy import deepcopy
from rclpy.time import Time
import math
import tf2_ros

class Optitrack(Node):
  def __init__(self, debug=False):
    super().__init__('optitrack_node')
    self.publisher_pose = self.create_publisher(PoseStamped, '/optitrack/pose', 10)
    self.publisher_twist = self.create_publisher(Twist, '/optitrack/twist', 10)
    #not used
    #node = rclpy.create_node('optitrack_node')
    #self.rate = node.create_rate(200) 
    
    self.R_w = np.array([[1.0,0.0,0.0], 
                        [0.0,0.0,-1.0],
                        [0.0,1.0,0.0]])

    self.publisher_frequency = 200.0  # hz
    self.pub_timer = self.create_timer(1/self.publisher_frequency, self.pub_timer_callback)
    self.actual_pose = PoseStamped()
    self.feasible_pose = PoseStamped()
    self.feasible_pose_old =PoseStamped()
    self.twist_msg = Twist()
    self.vel = np.zeros((3))
    self.omega = np.zeros((3))
    self.time = 0
    self.debug = debug
    self.calls = 0
    self.total_calls = 0
    self.new_data = False
    if self.debug:
      self.debug_timer = self.create_timer(1.0, self.debug_timer_callback)

    self.get_logger().info('Optitrack node running - version 2.2')

    streamingClient = NatNetClient(ver=(3, 2, 0, 0), quiet=True)
    streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
    streamingClient.run()

  def euler_from_quaternion(self, quaternion):

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
  
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
  
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
  
    return roll_x, pitch_y, yaw_z 

  def quaternion_from_euler(self, ai, aj, ak, axes='szyx'):
      """Return quaternion from Euler angles and axis sequence.

      ai, aj, ak : Euler's roll, pitch and yaw angles
      axes : One of 24 axis sequences as string or encoded tuple

      >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
      >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
      True

      """
      # epsilon for testing whether a number is close to zero
      _EPS = numpy.finfo(float).eps * 4.0

      # axis sequences for Euler angles
      _NEXT_AXIS = [1, 2, 0, 1]

      # map axes strings to/from tuples of inner axis, parity, repetition, frame
      _AXES2TUPLE = {
          'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
          'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
          'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
          'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
          'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
          'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
          'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
          'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

      _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

      try:
          firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
      except (AttributeError, KeyError):
          _ = _TUPLE2AXES[axes]
          firstaxis, parity, repetition, frame = axes

      i = firstaxis
      j = _NEXT_AXIS[i+parity]
      k = _NEXT_AXIS[i-parity+1]

      if frame:
          ai, ak = ak, ai
      if parity:
          aj = -aj

      ai /= 2.0
      aj /= 2.0
      ak /= 2.0
      ci = math.cos(ai)
      si = math.sin(ai)
      cj = math.cos(aj)
      sj = math.sin(aj)
      ck = math.cos(ak)
      sk = math.sin(ak)
      cc = ci*ck
      cs = ci*sk
      sc = si*ck
      ss = si*sk

      quaternion = Quaternion() 
      if repetition:
          quaternion.z = cj*(cs + sc)
          quaternion.y = sj*(cc + ss)
          quaternion.x = sj*(cs - sc)
          quaternion.w = cj*(cc - ss)
      else:
          quaternion.z = cj*sc - sj*cs
          quaternion.y = cj*ss + sj*cc
          quaternion.x = cj*cs - sj*sc
          quaternion.w = cj*cc + sj*ss
      if parity:
          quaternion.y *= -1

      return quaternion



  def computeOmega(self, q, q_old, dt):
     q1 = np.array([q.w, q.x, q.y, q.z])
     q2 = np.array([q_old.w, -q_old.x, -q_old.y, -q_old.z])
 
     work = (2/dt) * self.quatMultiply(q1, q2)
     #take immaginary part
     omega = work[1:]
     return omega

  def quatMultiply(self, q, r):
    s1 = q[0]
    s2 = r[0]
    v1 = q[1:]
    v2 = r[1:]


    # Calculate vector portion of quaternion product
    vec = s1*v2 + s2*v1 + np.cross(v1,v2)
    
    # Calculate scalar portion of quaternion product
    scalar = s1*s2 - v1.dot(v2)
        
    qout = np.concatenate(([scalar], vec))
    return qout
  
  def getTimeInterval(self, timestamp1, timestamp2):
    nanosec1 = float(timestamp1.nanosec) * 1e-9;
    time1 = float(timestamp1.sec) + nanosec1
    nanosec2 = float(timestamp2.nanosec) * 1e-9;
    time2 = float(timestamp2.sec) + nanosec2
    return time1- time2

  def debug_timer_callback(self):
    print("Hz: "+str(self.calls))
    
    self.calls = 0

  def pub_timer_callback(self):
 
    if self.total_calls>=1:#this publishes only if it has received at least 1 message in the other thread

          
      if self.new_data:       
        self.vel, self.omega[2], _, _, _ = self.computeNumericalDiff(self.actual_pose, self.feasible_pose, self.publisher_frequency)
        self.new_data = False
        self.feasible_pose = deepcopy(self.actual_pose)
      else:
        #do extrapolation if message has not changed is non zero (eg skip initial point)
        self.vel, self.omega[2], r,p,y = self.computeNumericalDiff(self.feasible_pose, self.feasible_pose_old, self.publisher_frequency)
        self.feasible_pose_old = deepcopy(self.feasible_pose)

        #extrapolate feasible pose
        self.feasible_pose.header.stamp = self.get_clock().now().to_msg()
        self.feasible_pose.pose.position.x += self.vel[0]*self.publisher_frequency
        self.feasible_pose.pose.position.y += self.vel[1]*self.publisher_frequency
        self.feasible_pose.pose.position.z += self.vel[2]*self.publisher_frequency

        #integrate quatertion
        self.feasible_pose.pose.orientation = self.quaternion_from_euler(r, p, y+self.omega[2]*self.publisher_frequency)

      self.twist_msg.linear.x = self.vel[0]
      self.twist_msg.linear.y = self.vel[1]
      self.twist_msg.linear.z = self.vel[2]
      self.twist_msg.angular.x = self.omega[0]
      self.twist_msg.angular.y = self.omega[1]
      self.twist_msg.angular.z = self.omega[2]
      self.publisher_twist.publish(self.twist_msg)

      self.publisher_pose.publish(self.feasible_pose)

    
  def computeNumericalDiff(self, act, old, freq):
      old_pos = np.array([old.pose.position.x, old.pose.position.y, old.pose.position.z])
      act_pos = np.array([act.pose.position.x, act.pose.position.y, act.pose.position.z])
      r_new,p_new,y_new = self.euler_from_quaternion(act.pose.orientation)
      r_old,p_old,y_old = self.euler_from_quaternion(old.pose.orientation)
      vel = (act_pos - old_pos)*freq
      omega = (y_new - y_old)*freq
      return vel, omega, r_new,p_new,y_new

  def receiveRigidBodyFrame(self, id, position, rotation):
    if (id==TRACKED_ROBOT_ID):
 
      # retrieve as originally published by the optitrack
      self.actual_pose.header.frame_id = "tag"
      self.actual_pose.header.stamp = self.get_clock().now().to_msg()


      self.actual_pose.pose.position.x = self.R_w[0,0]*position[0] + self.R_w[0,1]*position[1]  + self.R_w[0,2]*position[2]
      self.actual_pose.pose.position.y = self.R_w[1,0]*position[0] + self.R_w[1,1]*position[1]  + self.R_w[1,2]*position[2]
      self.actual_pose.pose.position.z = self.R_w[2,0]*position[0] + self.R_w[2,1]*position[1]  + self.R_w[2,2]*position[2]
      #In streamed NatNet data packets, orientation data is represented in the quaternion format (qx, qy, qz, qw). I   ] # q_w
      self.actual_pose.pose.orientation.w = rotation[3] # q_w
      self.actual_pose.pose.orientation.x = rotation[0] # q_x
      self.actual_pose.pose.orientation.y = rotation[2] # q_z
      self.actual_pose.pose.orientation.z = rotation[1] # -q_y

      #flip quaternion if long path 
      old_quat = np.array([self.feasible_pose.pose.orientation.w, self.feasible_pose.pose.orientation.x, self.feasible_pose.pose.orientation.y, self.feasible_pose.pose.orientation.z])
      new_quat = np.array([self.actual_pose.pose.orientation.w, self.actual_pose.pose.orientation.x, self.actual_pose.pose.orientation.y, self.actual_pose.pose.orientation.z])
      if old_quat.dot(new_quat) < 0:
          #flip new quat
          self.actual_pose.pose.orientation.w *= -1;
          self.actual_pose.pose.orientation.x *= -1;
          self.actual_pose.pose.orientation.y *= -1;
          self.actual_pose.pose.orientation.z *= -1;

      #check discontinuities wrt previous TODO
      # if  not( abs(self.feasible_pose.pose.orientation.w - self.actual_pose.pose.orientation.w)>0.4 or  
      #     abs(self.feasible_pose.pose.orientation.x - self.actual_pose.pose.orientation.x)>0.4 or 
      #     abs(self.feasible_pose.pose.orientation.y - self.actual_pose.pose.orientation.y)>0.4 or
      #     abs(self.feasible_pose.pose.orientation.z - self.actual_pose.pose.orientation.z)>0.4):
      #     # compute twists
      #     old_pos = np.array([self.feasible_pose.pose.position.x, self.feasible_pose.pose.position.y, self.feasible_pose.pose.position.z])
      #     act_pos = np.array([self.actual_pose.pose.position.x, self.actual_pose.pose.position.y, self.actual_pose.pose.position.z])
      #     dt=self.getTimeInterval(self.actual_pose.header.stamp, self.feasible_pose.header.stamp)
      #     self.vel = (act_pos - old_pos)/dt
      #     self.omega = self.computeOmega(self.actual_pose.pose.orientation, self.feasible_pose.pose.orientation, dt)
      #     # update the value if there are no discontiuities
      #     self.feasible_pose = deepcopy(self.actual_pose)

      #this cannot be zero
      #dt=self.getTimeInterval(self.actual_pose.header.stamp, self.actual_pose_old.header.stamp)
      
      # if dt>0.: #do the computation only when dt is non zero (eg skip initial point)
      #   old_pos = np.array([self.actual_pose_old.pose.position.x, self.actual_pose_old.pose.position.y, self.actual_pose_old.pose.position.z])
      #   act_pos = np.array([self.actual_pose.pose.position.x, self.actual_pose.pose.position.y, self.actual_pose.pose.position.z])
      #   r,p,y_new = self.euler_from_quaternion(self.actual_pose.pose.orientation)
      #   r,p,y_old = self.euler_from_quaternion(self.actual_pose_old.pose.orientation)
      #   self.vel = (act_pos - old_pos)*self.publisher_frequency
      #   self.omega[2] = (y_new - y_old)*self.publisher_frequency
      #   self.actual_pose_old = deepcopy(self.actual_pose)
      self.new_data = True

    
      # if self.debug:
      #   self.time+=dt
      #   print("Optitrack time", self.time)

      self.calls = self.calls + 1
      self.total_calls = self.total_calls + 1

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

  # try:
  #   while rclpy.ok():
  #       #not used
  #       # optitrack.pub_timer_callback()
  #       # rclpy.spin_once(optitrack)
  #       # optitrack.rate.sleep()        
  # except KeyboardInterrupt:
  #   pass

  rclpy.spin(optitrack)
  optitrack.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
