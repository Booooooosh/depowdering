# Copyright CERLAB @ CMU
# This script spawn a TCP listener for receiving
# 3D point cloud data from the camera
#
# Some assumption:
# 1. Currently only support receiving data from
#    one camera at a time

import socket
import select
import threading
import json
import fcntl
import struct

import os
import time
import logging
import sys

import rospy

class BlackboxListener:
  def __init__(self, localhost = True):
    # Socket param
    self.localhost = localhost
    self.iface = rospy.get_param('~rwr_interface', ['wlan0', 'eth0'])
    self.ip = None
    self.port = rospy.get_param('~rwr_port', 8080)
    self.buffer_size = rospy.get_param('~rwr_buffer_size', 1024)
    self.thread_timeout = rospy.get_param('~rwr_thread_timeout', 60.0)
    self.refreash_threashold = rospy.get_param('~rwr_refreash_accept_list', 10.0)
    self.recv_delay_tolerance = rospy.get_param('~rwr_recv_delay_tolerance', 3.0)
    # Thread control
    self.thread = None
    self.thread_abort = threading.Event()
    self.data_ready = threading.Semaphore(value = 0)
    # Data
    self.data = dict()
    rospy.loginfo('[Listener] Initiated.')

  def reset(self):
    # Calling this function will reset BlackboxListener class
    if self.thread != None:
      rospy.logwarn('[Listener] Thread still working, can not reset...')
      return False
    else:
      self.thread = None
      self.thread_abort = threading.Event()
      self.data_ready = threading.Semaphore(value = 0)
      self.data = dict()
      rospy.loginfo('[Listener] Reset Done.')
      return True

  def start(self):
    self.thread = threading.Thread(name = 'blackbox_listener', target = self.listener, args = ())
    self.thread.start()
    rospy.loginfo('[Listener] Worker thread spawned.')
    return self.thread

  def __analyze_ip_addr(self, sock):
    ip_addr = None
    for each_iface in self.iface:
      try:
        ip_addr = socket.inet_ntoa(fcntl.ioctl(sock.fileno(),
                                    0x8915,  # SIOCGIFADDR
                                    struct.pack('256s', each_iface[:15])
                                    )[20:24])
      except:
        continue
      else:
        return ip_addr
    # No ip address available
    return None

  def listener(self, backlog = 5):
    try:
      listener_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      # Set property @ non-block socket for listener
      listener_sock.setblocking(False)
      # Set property @ reusable socket
      listener_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      # Find out which ip_addr we want to use
      if self.localhost == True:
        # Using local host
        self.ip = '127.0.0.1'
      else:
        # Determine the current ip addr
        self.ip = self.__analyze_ip_addr(listener_sock)
        if not self.ip:
          self.ip = '127.0.0.1'
      listener_sock.bind((self.ip, self.port))
      rospy.loginfo('\t[Listener Worker] Bond to ({0}: {1})'.format(self.ip, self.port))
      listener_sock.listen(backlog)

      # Setting up receving sock list
      timeout = False; wait_seq = 0
      inputs = [listener_sock]
      outputs = []

      # Select
      while inputs and not self.thread_abort.isSet():
        if not timeout:
          rospy.loginfo('\t[Listener Worker] Current # of robots: %d' % (len(inputs) - 1))
        else:
          rospy.loginfo('\t[Still Waiting] Have been waiting {0} sec for robots to connect...'.format(self.refreash_threashold * wait_seq))
        [incoming, outcoming, exceptional] = select.select(inputs, outputs, inputs, self.refreash_threashold)

        # Check for timeout
        if not incoming and not outcoming and not exceptional:
          timeout = True
          wait_seq += 1
        else:
          timeout = False
          wait_seq = 0

        # Handle incoming socket request
        for each_sock in incoming:
          if each_sock is listener_sock:
            # Ready to accept a client
            [remote_sock, remote_addr] = each_sock.accept()
            rospy.loginfo('\t[Listener Worker] Remote conn detected: {0} : {1}.'.format(remote_addr[0], remote_addr[1]))
            remote_sock.settimeout(self.recv_delay_tolerance)
            inputs.append(remote_sock)
          else:
            # Fetch data from camera
            raw_data = each_sock.recv(self.buffer_size)
            # I do not care about the data right now
            # Indicate that data is ready
            self.data_ready.release()
            rospy.loginfo('\t[Listener Worker] Received signal from the robot.')

        # Handle exceptional socket
        for each_sock in exceptional:
          rospy.logerr('\t[Listener Worker] Exceptional socket {0}'.format(each_sock.getpeername()))
          inputs.remove(each_sock)
          each_sock.close()

      # Abort thread
      rospy.loginfo('\t[Listener Worker] Exiting worker thread...')
      # Close all sockets
      for each_sock in inputs:
        each_sock.close()

    except socket.error as socket_err:
      rospy.logerr('\t[Listener Worker] Socket Error({0}): {1}.'.format(socket_err.errno, socket_err.strerror))
    except socket.timeout as timeout_err:
      rospy.logerr('\t[Listener Worker] Timeout Error({0}): {1}.'.format(timeout_err.errno, timeout_err.strerror))
    except select.error as select_err:
      rospy.logerr('\t[Listener Worker] Select Error({0}): {1}.'.format(select_err.errno, select_err.strerror))
    else:
      rospy.loginfo('\t[Listener Worker] Exit.')

  def end(self):
    rospy.loginfo('[Listener] Killing worker thread...')
    self.thread_abort.set()
    self.thread.join(self.thread_timeout)
    if self.thread.isAlive():
      rospy.logerr('[Listener] Worker thread did not respond after %lf seconds.' % self.thread_timeout)
      return False
    else:
      self.thread = None
      self.reset()
      rospy.loginfo('[Listener] Done.')
      return True

  def retrieve_data(self, blocking = True):
    # Wait for semaphore
    if self.thread == None or self.thread.isAlive() == False:
      rospy.logerr('[Listener] Worker thread not started yet.')
      return (False, None)
    else:
      success = self.data_ready.acquire(blocking)
      return (success, self.data)

# Here is the unit test for BlackboxListener class
# When you hit ENTER, test will end
if __name__ == '__main__':
  try:
    rospy.loginfo("----------------------------------------")
    rospy.loginfo("You're in the unit test for listener.")
    rospy.loginfo("Type anything and Press ENTER to receive data.")
    rospy.loginfo("Press ENTER ALONE to exit.")
    rospy.loginfo("----------------------------------------")
    listener = BlackboxListener(localhost = False)
    listener.start()
    while raw_input() != '':
      data = listener.retrieve_data()
      print data
      time.sleep(0.1)
    listener.end()
  except KeyboardInterrupt:
    print 'Ctrl-C detected.'
    listener.end()
    sys.exit(0)
