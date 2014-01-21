# Copyright 2008 Mike Wakerly <opensource@hoho.com>
#
# This file is part of the Kegboard package of the Kegbot project.
# For more information on Kegboard or Kegbot, see http://kegbot.org/
#
# Kegboard is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# Kegboard is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Kegboard.  If not, see <http://www.gnu.org/licenses/>.

"""Python interfaces to a Kegboard device."""

import glob
import itertools
import os
import struct
import time

import gflags
import select
import serial

from . import crc16
from .message import *

GLOB_PATHS = (
  '/dev/ttyUSB*',
  '/dev/ttyACM*',
  '/dev/cu.usbserial*',
  '/dev/tty.usbmodem*'
)

def find_devices():
  paths = []
  for p in GLOB_PATHS:
    paths += glob.glob(p)
  return paths

_GLOBS = find_devices()

if _GLOBS:
  _DEFAULT_PORT = _GLOBS[0]
else:
  _DEFAULT_PORT = '/dev/ttyUSB0'

FLAGS = gflags.FLAGS

gflags.DEFINE_string('kegboard_device', _DEFAULT_PORT,
    'An explicit device file (eg /dev/ttyUSB0) on which to listen for kegboard '
    'packets.')

gflags.DEFINE_integer('kegboard_speed', 115200,
    'Baud rate of device at --kegboard_device')

gflags.DEFINE_boolean('verbose', os.environ.get('VERBOSE') is not None,
    'Generate extra logging information.',
    allow_override=True)



class KegboardError(Exception):
  """Generic error with Kegboard"""


class UnknownMessageError(KegboardError):
  """Message id is not known"""


def wait_for_kegboard(interval=0.1, timeout=None):
  elapsed = 0
  while True:
    paths = find_devices()
    if paths:
      return Kegboard(paths[0])

    elapsed += interval
    if timeout is not None and elapsed >= timeout:
      return None
    time.sleep(interval)


class Kegboard:
  def __init__(self, device_path, speed=None):
    self.device_path = device_path
    self.incomplete_message = ""
    if not speed:
      speed = FLAGS.kegboard_speed
    self.speed = speed
    self.fd = None

  def __str__(self):
    return '<Kegboard path=%s speed=%s>' % (self.device_path, self.speed)

  def open(self):
    """Opens the backing device; must be called before any operations."""
    if self.fd:
      raise IOError('Already open!')
    if not os.path.isfile(self.device_path):
      self.fd = serial.Serial(self.device_path, self.speed, timeout=0.1)
      self.fd.flushInput()
    else:
      self.fd = open(self.device_path, 'rb')

  def close(self):
    """Closes the backing device."""
    self._assert_open()
    self.fd.close()
    self.fd = None
    self.incomplete_message = ""

  def close_quietly(self):
    """Similar to `close()`, but swallows any errors."""
    try:
      self.close()
    except IOError:
      pass

  def read_message_nonblock(self):
    """Immedaitely returns a message if available, None otherwise."""
    self._assert_open()

    while True:
      # Since we also support 'plain' fds, cannot use serial.inWaiting()
      rr, wr, er = select.select([self.fd], [], [], 0)
      if not rr:
        break

      c = self.fd.read(1)

      if self.incomplete_message is None:
        if c == '\n':
          # Reset.
          self.incomplete_message = ''
        continue

      self.incomplete_message += c

      if len(self.incomplete_message) >= KBSP_MAXLEN:
        # Packet too big; mark corrupt.
        self.incomplete_message = None

      elif not KBSP_PREFIX.startswith(self.incomplete_message[:len(KBSP_PREFIX)]):
        # Bad packet start; mark corrupt.
        self.incomplete_message = None

      elif self.incomplete_message[-2:] == KBSP_TRAILER:
        # Packet ended! Check it.
        bytes = self.incomplete_message
        self.incomplete_message = ''

        header = bytes[:12]
        payload = bytes[12:-4]
        trailer = bytes[-4:]
        crcd_bytes = bytes[:-2]
        checked_crc = crc16.crc16_ccitt(crcd_bytes)

        message_id, message_len = struct.unpack('<HH', header[8:])
        try:
          return get_message_by_id(message_id, payload)
        except UnknownMessageError, e:
          continue

      else:
        # Just continue.
        continue

    return None

  def drain_messages(self):
    """Immediately returns all available messages without blocking.

    This method is a convenience wrapper around `read_message_nonblock()`.
    """
    self._assert_open()
    ret = []
    while True:
      m = self.read_message_nonblock()
      if not m:
        break
      ret.append(m)
    return ret

  def read_message(self, timeout=None, interval=0.1):
    """Blocks until a message is available, returning it.

    If `timeout` given, the method will return None after this many seconds
    have elapsed without reading a message.
    """
    self._assert_open()
    elapsed = 0
    while True:
      m = self.read_message_nonblock()
      if m:
        return m

      elapsed += interval
      if timeout is not None and elapsed >= timeout:
        return None
      time.sleep(interval)

  def write_message(self, message):
    """Send a message to the device."""
    self._assert_open()
    return self.fd.write(message.ToBytes())

  def _assert_open(self):
    if not self.fd:
      raise IOError('Kegboard not open; call open() first.')