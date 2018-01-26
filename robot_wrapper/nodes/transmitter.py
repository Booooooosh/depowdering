# Copyright CERLAB @ CMU
# This script spawn a TCP transmitter for 
# sending trajectory to the robot
#
# Some assumption:
# 1. The output data is an array consisting of
#    continuous 3D point for roobt execution.

import os
import sys
import socket
import json
import threading
import time
import logging

import rospy

class BlackboxTransmitter():
  def __init__(self, localhost = True):
    # Socket param
    self.localhost = localhost
    self.ip = rospy.get_param('~rws_ip_addr', '127.0.0.1')
    self.port = rospy.get_param('~rws_port', 8081)
    self.thread_timeout = rospy.get_param('~rws_thread_timeout', 60.0)
    self.event_timeout = rospy.get_param('~rws_event_timeout', 5.0)
    # Thread control
    self.thread = None
    self.thread_abort = threading.Event()
    self.transmit_data = threading.Event()
    # Data
    self.data = dict()
    rospy.loginfo('[Transmitter] Initiated.')

  # Function @ data2json
  # Transfer a dictionary-like data structure
  # into JSON object. NOTE: We assume that the
  # input data is NOT a JSON object.
  def __data2json(self, data):
    try:
      json_data = json.dumps(data, allow_nan = True, sort_keys = True)
    except TypeError as type_err:
      rospy.logerr('\t[Tranmitter Worker] JSON conversion type error: %s' % str(type_err))
      return None
    except OverflowError as overflow_err:
      rospy.logerr('\t[Tranmitter Worker] JSON conversion circular reference detected.')
      return None
    except ValueError as val_err:
      rospy.logerr('\t[Tranmitter Worker] JSON conversion value error: NaN, Inf, -Inf detected.')
      return None
    except:
      rospy.logerr('\t[Tranmitter Worker] OMG, What happened? JSON conversion unexpected error.')
      return None
    else:
      logging.debug('\t[Tranmitter Worker] data --> JSON object conversion successful!')
      return json_data

  def reset(self):
    # Calling this function will reset BlackboxTransmitter class
    if self.thread != None:
      rospy.logwarn('[Transmitter] Worker thread still alive, can not reset.')
      return False
    else:
      self.thread = None
      self.thread_abort = threading.Event()
      self.data_ready = threading.Event()
      self.data = dict()
      rospy.loginfo('[Transmitter] Reset.')
      return True

  def start(self):
    self.thread = threading.Thread(name = 'blackbox_transmitter', target = self.transmit, args = ())
    self.thread.start()
    rospy.loginfo('[Transmitter] Worker thread spawned.')
    return self.thread

  def transmit(self):
    while not self.thread_abort.isSet():
      try:
        transmit_sock_connected = False
        transmit_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.localhost == True:
          self.ip = '127.0.0.1'

        ## Connecting to the server ##
        while transmit_sock.connect_ex((self.ip, self.port)) != 0 and not self.thread_abort.isSet():
          rospy.loginfo('\t[Tranmitter Worker] Still trying to connect to {0}:{1}...'.format(self.ip, self.port))
          time.sleep(20.0)
        if not self.thread_abort.isSet():
          rospy.loginfo('\t[Tranmitter Worker] Connected to {0}:{1}!'.format(self.ip, self.port))
          transmit_sock_connected = True

        ## Sending data ##
        while not self.thread_abort.isSet():
          # Waiting for signal to send data
          self.transmit_data.wait(self.event_timeout)
          if self.transmit_data.isSet():
            # Dump into JSON and transmit
            self.transmit_data.clear()
            data_json = self.__data2json(self.data)
            # Check whether JSON conversion is successful
            if data_json == None:
              continue
            # Add length of data in front
            data_json = str(len(data_json)) + '>' + data_json
            # Sending data
            bytes_sent = 0
            bytes_total = len(data_json)
            while bytes_sent < bytes_total:
              sent = transmit_sock.send(data_json[bytes_sent :])
              if sent == 0:
                raise RuntimeError('\t[Tranmitter Worker] TCP connection error.')
              bytes_sent += sent
            rospy.loginfo('\t[Tranmitter Worker] Frame({0} bytes) sent.'.format(bytes_sent))
          else:
            continue
      except socket.error as socket_err:
        rospy.logerr('\t[Tranmitter Worker] Socket Error({0}): {1}.'.format(socket_err.errno, socket_err.strerror))
        rospy.loginfo('\t[Tranmitter Worker] Try reconnecting to the server...')
        # try re-connecting to the server
        transmit_sock_connected = False
        transmit_sock.close()
        transmit_sock = None
        continue
      except socket.timeout as timeout_err:
        rospy.logerr('\t[Tranmitter Worker] Timeout Error({0}): {1}.'.format(timeout_err.errno, timeout_err.strerror))
        rospy.loginfo('\t[Tranmitter Worker] Try reconnecting to the server...')
        transmit_sock_connected = False
        transmit_sock.close()
        transmit_sock = None
        continue
      except RuntimeError as err:
        rospy.logerr('\t[Tranmitter Worker] Runtime Error detected.')
        rospy.loginfo('\t[Tranmitter Worker] Try reconnecting to the server...')
        transmit_sock_connected = False
        transmit_sock.close()
        transmit_sock = None
        continue
      else:
        rospy.loginfo('\t[Tranmitter Worker] Exit.')
        if transmit_sock_connected:
          transmit_sock.shutdown(socket.SHUT_RDWR)
        transmit_sock.close()


  def end(self):
    rospy.loginfo('[Transmitter] Killing worker thread...')
    self.thread_abort.set()
    self.thread.join(self.thread_timeout)
    if self.thread.isAlive():
      rospy.logerr('[Transmitter] Worker failed to response after %lf seconds.' % self.thread_timeout)
      return False
    else:
      self.thread = None
      self.reset()
      rospy.loginfo('[Transmitter] Done.')
      return True

  def send_data(self, data):
    if not data:
      rospy.logerr('[Transmitter] Empty data detected.')
      return False
    # A shallow copy of the data
    self.data = data
    rospy.loginfo('[Transmitter] Sending data...')
    self.transmit_data.set()
    return True

if __name__ == '__main__':
  try:
    rospy.loginfo("----------------------------------------")
    rospy.loginfo("You're in the unit test for transmitter.")
    rospy.loginfo("Type anything and Press ENTER to send data.")
    rospy.loginfo("Press ENTER ALONE to exit.")
    rospy.loginfo("----------------------------------------")
    transmitter = BlackboxTransmitter()
    transmitter.start()
    while raw_input() != "":
      # Create a test data dictionary
      print 'Prepare to send data.'
      data = {'size': 1, 'data': [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]}
      transmitter.send_data(data)
      time.sleep(5.0)
  except KeyboardInterrupt:
    print 'Ctrl-C detected.'
  finally:
    transmitter.end()
    sys.exit(0)
