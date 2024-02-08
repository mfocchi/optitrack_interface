# WIFI CONFIG
# SSID: OptiTrack
# PSW: 60A84A244BECD

import socket
import struct
import numpy
from threading import Thread
from time import time
from math import atan2, asin

# Create structs for reading various object types to speed up parsing.
Vector = struct.Struct('<fff')
Quatern = struct.Struct('<ffff')
IntValue = struct.Struct('<i')
FloatValue = struct.Struct('<f')
DoubleValue = struct.Struct('<d')



class NatNetClient:
  # def __init__(self, ver=(3, 0, 0, 0), server_ip="192.168.2.3", quiet=True):
  def __init__(self, ver=(3, 0, 0, 0), server_ip="192.168.1.3", quiet=True):
    self.__natNetStreamVersion = ver
    self.serverIPAddress = server_ip
    self.multicastAddress = "239.255.42.99"
    self.commandPort = 1510
    self.dataPort = 1511
    # Callbacks
    self.rigidBodyListener = None
    self.newFrameListener = None

  # Client/server message ids
  NAT_PING = 0
  NAT_PINGRESPONSE = 1
  NAT_REQUEST = 2
  NAT_RESPONSE = 3
  NAT_REQUEST_MODELDEF = 4
  NAT_MODELDEF = 5
  NAT_REQUEST_FRAMEOFDATA = 6
  NAT_FRAMEOFDATA = 7
  NAT_MESSAGESTRING = 8
  NAT_DISCONNECT = 9
  NAT_UNRECOGNIZED_REQUEST = 100

  # ============================= Data unpacking ============================ #
  def __unpackMarkerSet(self, data):
    offset = 0
    # Model name
    modelName, separator, remainder = bytes(data[offset:]).partition(b'\0')
    offset += len(modelName) + 1
    # Marker count
    markerCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    # Markers
    for j in range(markerCount):
      pos = Vector.unpack(data[offset:offset + 12])
      offset += 12
    return offset

  def __unpackRigidBody(self, data):
    offset = 0

    # ID (4 bytes)
    rb_id = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    # Position and orientation
    pos = Vector.unpack(data[offset:offset + 12])
    offset += 12
    rot = Quatern.unpack(data[offset:offset + 16])
    offset += 16

    # After Version 3.0, marker data is in description
    ver = self.__natNetStreamVersion
    if ver[0] < 3:
      # Marker count (4 bytes)
      markerCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
      offset += 4
      markerCountRange = range(markerCount)

      # Marker positions
      for i in markerCountRange:
        pos = Vector.unpack(data[offset:offset + 12])
        offset += 12

      if ver[0] >= 2:
        # Marker ID's
        for i in markerCountRange:
          marker_id = int.from_bytes(data[offset:offset + 4], byteorder='little')
          offset += 4

        # Marker sizes
        for i in markerCountRange:
          size = FloatValue.unpack(data[offset:offset + 4])
          offset += 4

    # NatNet version 2.0 and later
    if ver[0] >= 2:
      markerError, = FloatValue.unpack(data[offset:offset + 4])
      offset += 4

    # Version 2.6 and later
    if (ver[0] == 2 and ver[1] >= 6 or ver[0] > 2):
      param, = struct.unpack('h', data[offset:offset + 2])
      offset += 2
      trackingValid = (param & 0x01) != 0

    # Send information to any listener.
    if self.rigidBodyListener is not None:
      self.rigidBodyListener(rb_id, pos, rot)

    return offset

  def __unpackSkeleton(self, data):
    offset = 0
    # Skeleton ID
    id = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    # Rigid body count
    rigidBodyCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    # Rigid bodies
    for j in range(rigidBodyCount):
      offset += self.__unpackRigidBody(data[offset:])

    return offset

  def __unpackLabeledMarker(self, data):
    offset = 0
    id = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    model_id = id >> 16
    marker_id = id & 0x0000ffff
    pos = Vector.unpack(data[offset:offset + 12])
    offset += 12
    size = FloatValue.unpack(data[offset:offset + 4])
    offset += 4

    # Version 2.6 and later
    ver = self.__natNetStreamVersion
    if ver[0] == 2 and ver[1] >= 6 or ver[0] > 2:
      param, = struct.unpack('h', data[offset:offset + 2])
      offset += 2
      occluded = (param & 0x01) != 0
      pointCloudSolved = (param & 0x02) != 0
      modelSolved = (param & 0x04) != 0
      if ver[0] >= 3:
        hasModel = (param & 0x04) != 0
        unlabeled = (param & 0x10) != 0
        activeMarker = (param & 0x20) != 0

    # Version 3.0 and later
    if ver[0] >= 3:
      residual, = FloatValue.unpack(data[offset:offset + 4])
      offset += 4

    return offset

  def __unpackForcePlate(self, data):
    offset = 0
    # Force plate ID
    forcePlateID = int.from_bytes(data[offset:offset + 4],
                                  byteorder='little')
    offset += 4
    # Channel Count
    forcePlateChannelCount = int.from_bytes(data[offset:offset + 4],
                                            byteorder='little')
    offset += 4
    # Channel Data
    for j in range(forcePlateChannelCount):
      forcePlateChannelFrameCount = int.from_bytes(data[offset:offset + 4],
                                                   byteorder='little')
      offset += 4
      for k in range(forcePlateChannelFrameCount):
        forcePlateChannelVal = int.from_bytes(data[offset:offset + 4],
                                              byteorder='little')
        offset += 4
    return offset

  def __unpackDevice(self, data):
    offset = 0
    # ID
    deviceID = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    # Channel Count
    deviceChannelCount = int.from_bytes(data[offset:offset + 4],
                                        byteorder='little')
    offset += 4
    # Channel Data
    for j in range(deviceChannelCount):
      deviceChannelFrameCount = int.from_bytes(data[offset:offset + 4],
                                               byteorder='little')
      offset += 4
      for k in range(deviceChannelFrameCount):
        deviceChannelVal = int.from_bytes(data[offset:offset + 4],
                                          byteorder='little')
        offset += 4
    return offset

  def __unpackMocapData(self, data):
    data = memoryview(data)
    offset = 0

    # Frame number (4 bytes)
    frameNumber = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    
    # ================ Marker sets
    markerSetCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    for i in range(markerSetCount):
      offset += self.__unpackMarkerSet(data[offset:])

    # ================ Unlabeled markers (DEPRECATED)
    ver = self.__natNetStreamVersion
    unlabeledMarkersCount = int.from_bytes(data[offset:offset + 4],
                                           byteorder='little')
    offset += 4
    if ver[0] < 3:
      for i in range(unlabeledMarkersCount):
        pos = Vector.unpack(data[offset:offset + 12])
        offset += 12
    else:
      # Just skip them
      offset += 12 * unlabeledMarkersCount

    # ================ Rigid bodies
    rigidBodyCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    for i in range(rigidBodyCount):
      offset += self.__unpackRigidBody(data[offset:])

    # ================ Skeletons (Version 2.1 and later)
    skeletonCount = 0
    if ver[0] == 2 and ver[1] > 0 or ver[0] > 2:
      skeletonCount = int.from_bytes(data[offset:offset + 4],
                                     byteorder='little')
      offset += 4
      for i in range(skeletonCount):
        offset += self.__unpackSkeleton(data[offset:])

    # ================ Labeled markers (Version 2.3 and later)
    labeledMarkerCount = 0
    if ver[0] == 2 and ver[1] >= 3 or ver[0] > 2:
      labeledMarkerCount = int.from_bytes(data[offset:offset + 4],
                                          byteorder='little')
      offset += 4
      for i in range(labeledMarkerCount):
        offset += self.__unpackLabeledMarker(data[offset:])

    # ================ Force Plate data (version 2.9 and later)
    if ver[0] == 2 and ver[1] >= 9 or ver[0] > 2:
      forcePlateCount = int.from_bytes(data[offset:offset + 4],
                                       byteorder='little')
      offset += 4
      for i in range(forcePlateCount):
        offset += self.__unpackForcePlate(data[offset:])

    # ================ Device data (version 2.11 and later)
    if ver[0] == 2 and ver[1] >= 11 or ver[0] > 2:
      deviceCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
      offset += 4
      for i in range(deviceCount):
        offset += self.__unpackDevice(data[offset:])

    # ================ Timing
    if ver[0] < 3:
      softwareLatency = FloatValue.unpack(data[offset:offset + 4])
      offset += 4

    # Timecode
    timecode = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    timecodeSub = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    # Timestamp (increased to double precision in 2.7 and later)
    if ver[0] == 2 and ver[1] >= 7 or ver[0] > 2:
      timestamp, = DoubleValue.unpack(data[offset:offset + 8])
      offset += 8
    else:
      timestamp, = FloatValue.unpack(data[offset:offset + 4])
      offset += 4

    # Hires Timestamp (Version 3.0 and later)
    if ver[0] >= 3:
      stampCameraExposure = int.from_bytes(data[offset:offset + 8],
                                           byteorder='little')
      offset += 8
      stampDataReceived = int.from_bytes(data[offset:offset + 8],
                                         byteorder='little')
      offset += 8
      stampTransmit = int.from_bytes(data[offset:offset + 8],
                                     byteorder='little')
      offset += 8

    # ================ Frame parameters
    isRecording = trackedModelsChanged = False
    if ver[0] >= 3:
      param, = struct.unpack('h', data[offset:offset + 2])
      offset += 2
      isRecording = (param & 0x01) != 0
      trackedModelsChanged = (param & 0x02) != 0

    # Send information to any listener
    if self.newFrameListener is not None:
      self.newFrameListener(frameNumber, markerSetCount, unlabeledMarkersCount,
                            rigidBodyCount, skeletonCount,
                            labeledMarkerCount, timecode, timecodeSub,
                            timestamp, isRecording, trackedModelsChanged)

  # ======================= Data description unpacking ====================== #

  def __unpackMarkerSetDescription(self, data):
    offset = 0

    name, separator, remainder = bytes(data[offset:]).partition(b'\0')
    offset += len(name) + 1

    markerCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    for i in range(markerCount):
      name, separator, remainder = bytes(data[offset:]).partition(b'\0')
      offset += len(name) + 1

    return offset

  def __unpackRigidBodyDescription(self, data):
    offset = 0

    # Rigid body name (NatNet 2.0 and later)
    if self.__natNetStreamVersion[0] >= 2:
      name, separator, remainder = bytes(data[offset:]).partition(b'\0')
      offset += len(name) + 1

    id = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    parentID = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    coord_offset = Vector.unpack(data[offset:offset + 12])
    offset += 12

    # Per-marker data (NatNet 3.0 and later)
    if self.__natNetStreamVersion[0] >= 3:
      n_markers = int.from_bytes(data[offset:offset + 4], byteorder='little')
      offset += 4

      for marker_idx in range(n_markers):
        pos = Vector.unpack(data[offset:offset + 12])
        offset += 12

      for marker_idx in range(n_markers):
        marker_required_label = int.from_bytes(data[offset:offset + 4],
                                               byteorder='little')
        offset += 4

    return offset

  def __unpackSkeletonDescription(self, data):
    offset = 0

    name, separator, remainder = bytes(data[offset:]).partition(b'\0')
    offset += len(name) + 1

    id = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    rigidBodyCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    for i in range(rigidBodyCount):
      offset += self.__unpackRigidBodyDescription(data[offset:])

    return offset

  def __unpackDataDescriptions(self, data):
    offset = 0
    datasetCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4

    for i in range(datasetCount):
      type = int.from_bytes(data[offset:offset + 4], byteorder='little')
      offset += 4
      if type == 0:
        offset += self.__unpackMarkerSetDescription(data[offset:])
      elif type == 1:
        offset += self.__unpackRigidBodyDescription(data[offset:])
      elif type == 2:
        offset += self.__unpackSkeletonDescription(data[offset:])

  # ================================ Threads ================================ #

  def __processMessage(self, data):
    messageID = int.from_bytes(data[0:2], byteorder='little')

    packetSize = int.from_bytes(data[2:4], byteorder='little')

    offset = 4
    if messageID == self.NAT_FRAMEOFDATA:
      self.__unpackMocapData(data[offset:])
    elif messageID == self.NAT_MODELDEF:
      self.__unpackDataDescriptions(data[offset:])
    elif messageID == self.NAT_PINGRESPONSE:
      name, _, _ = bytes(data[offset:]).partition(b'\0')
      offset += 256  # Skip the sending app's Name field
      offset += 4  # Skip the sending app's Version info
      self.__natNetStreamVersion = struct.unpack('BBBB',
                                                 data[offset:offset + 4])
      offset += 4
    elif messageID == self.NAT_RESPONSE:
      if packetSize == 4:
        commandResponse = int.from_bytes(data[offset:offset + 4],
                                         byteorder='little')
        offset += 4
      else:
        message, separator, remainder = bytes(data[offset:]).partition(b'\0')
        offset += len(message) + 1
    elif messageID == self.NAT_MESSAGESTRING:
      message, separator, remainder = bytes(data[offset:]).partition(b'\0')
      offset += len(message) + 1


  def __threadFunction(self, socket):
    while True:
      # print("DEBUG: getting messagges")
      # Block for input
      data, addr = socket.recvfrom(32768)  # 32k byte buffer size
      # print("DEBUG: " + str(len(data)))
      if len(data) > 0:
        self.__processMessage(data)

  # ================================ Sockets ================================ #
  def __createDataSocket(self, port):
    result = socket.socket(socket.AF_INET,  # Internet
                           socket.SOCK_DGRAM,
                           socket.IPPROTO_UDP)  # UDP
    result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # A hacky way to get the IP of the interface that connects to the Internet
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    my_ip = s.getsockname()[0]
    s.close()
    # Specify my_ip as the interface to subscribe to multicast through
    result.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP,
                      socket.inet_aton(self.multicastAddress)
                      + socket.inet_aton(my_ip))

    result.bind((self.multicastAddress, port))
    return result

  def __createCommandSocket(self):
    result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    result.bind(('', 0))
    result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return result

  # ================================= Main ================================== #
  def sendCommand(self, command, commandStr, socket, address):
    packetSize = 0
    if command == self.NAT_REQUEST:
      packetSize = len(commandStr) + 1

    data = command.to_bytes(2, byteorder='little')
    data += packetSize.to_bytes(2, byteorder='little')

    data += commandStr.encode('utf-8')
    data += b'\0'

    socket.sendto(data, address)

  def run(self):
    # Data socket and thread
    self.dataSocket = self.__createDataSocket(self.dataPort)
    if self.dataSocket is None:
      raise RuntimeError("Could not open data channel")
    dataThread = Thread(target=self.__threadFunction, args=(self.dataSocket,))

    # Command socket and thread
    self.commandSocket = self.__createCommandSocket()
    if self.commandSocket is None:
      raise RuntimeError("Could not open command channel")
    commandThread = Thread(target=self.__threadFunction,
                           args=(self.commandSocket,))

    dataThread.start()

