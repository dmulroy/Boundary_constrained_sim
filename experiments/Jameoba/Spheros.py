"""
This module contains classes for single and multiple Sphero Mini/Bolt to
be used with ROS.
"""

###################################################
from Sphero_mini.sphero_mini import sphero_mini
from bluepy.btle import BTLEInternalError, BTLEDisconnectError
from pysphero.core import Sphero as PySphero
from pysphero.driving import StabilizationIndex
from pysphero.sensor import SensorCommand, Quaternion, Attitude, \
	Accelerometer, AccelOne,  Locator, Velocity, Speed, CoreTime, \
	Gyroscope, AmbientLight
from pysphero.user_io import Color
from pysphero.packet import Packet
from pysphero.exceptions import PySpheroTimeoutError
from numpy import ndarray, zeros, ones
from enum import Enum
from time import time
###################################################


###################################################
# ------------------ Constants ------------------ #
###################################################


###################################################
MINI = 0
BOLT = 1
MAC_ADDRESS = {MINI: ["F1:F3:18:C4:B0:C7",
					  "F8:DE:83:C4:BC:40",
					  "D3:FC:0F:51:23:A6",
					  "E6:41:28:6D:4A:85",
					  "F4:69:1E:5A:40:53",
					  "CA:FC:91:5F:EF:C8",
					  "D9:60:B1:CD:98:CA"],
			   BOLT: ["D7:76:84:A6:AA:12",
					  "FC:B6:17:AD:5E:D4",
					  "F8:AD:2E:C4:76:1F",
					  "F9:15:B8:0C:F3:C4",
					  "C7:61:70:28:38:F6",
					  "EF:AA:45:E4:D3:D9",
					  "C8:F4:16:90:92:6C",
					  "FC:AA:EA:88:C2:8A",
					  "F6:EA:ED:72:50:70",
					  "E6:0F:87:57:6D:EB",
					  "D7:A4:A0:08:C0:E3",
					  "CE:85:C6:6F:AB:4F",
					  "DC:E8:A6:EC:DA:75",
					  "E8:82:68:53:B2:74",
					  "E9:78:63:9F:0A:67",
					  "EB:CD:F0:3B:51:CD"]}
# "C6:4F:EE:78:0E:DA"


class CustomPySpheroCommand(Enum):
	start_IR_broadcast = 0x27   
	start_evading = 0x33
	start_following = 0x28
	stop_IR_broadcast = 0x29
	stop_evading = 0x34
	stop_following = 0x32
###################################################


###################################################
# ----------------- Exceptions ------------------ #
###################################################


###################################################
class JAMEOBAException(Exception):
	"""Base class for all Jameoba exceptions.

	Parameters
	----------
	message : str
		Message to be displayed in the error.

	Attributes
	----------
	message : str
		Message to be displayed in the error.
	"""

	def __init__(self, message):
		"""Constructor for Jameoba exceptions.

		Parameters
		----------
		message : str
			Message to be displayed in the error.
		"""

		self.message = message

	def __str__(self):
		"""Method to transform the class into a str.

		Returns
		-------
		str
			The message that is being displayed.
		"""

		return self.message
###################################################


###################################################
class CantConnectError(JAMEOBAException):
	def __init__(self, message):
		"""Constructor for CantConnectError exceptions.

		Parameters
		----------
		message : str
			Message to be displayed in the error.
		"""

		super().__init__(message)
###################################################


###################################################
# --------------- Sphero classes ---------------- #
###################################################


###################################################
class Sphero:
	"""The super class of a single Sphero.

	Parameters
	----------
	tag_id : int
		The Apriltag ID of the Sphero.
	verbosity : int
		The verbosity level of the instance.

	Attributes
	----------
	id : int
		The Apriltag ID of the Sphero.
	verb : int
		The verbosity level of this class instance.
	connected : bool
		True if the Sphero is connected.
	is_follower : bool
		True if the Sphero is following.
	is_leader : bool
		True if the Sphero is leading.
	_heading : int
		The desired heading angle for the Sphero.
	_speed : int
		The desired speed for the Sphero.
	"""

	def __init__(self, tag_id=0, verbosity=1):
		"""Constructor for Sphero class.

		Parameters
		----------
		tag_id : int
			The Apriltag ID of the Sphero.
		verbosity : int
			The verbosity level of the instance.

		Yields
		------
		Sphero
			A Sphero instance.
		"""

		self.id = tag_id
		self.verb = verbosity
		self.connected = False
		self.is_follower = False
		self.is_leader = False
		self._heading = 0
		self._speed = 0

	def __del__(self):
		""" Destructor of the Sphero class. """

		print("\r  \r", end='')
		self.disconnect()

	def disconnect(self):
		""" Method to put the Sphero to sleep and disconnect it. """

		pass

	def send(self, speed=None, heading=None):
		"""Method to send speed and heading command to the Sphero.

		Parameters
		----------
		speed : int or None
			Desired speed of the Sphero.
		heading : int or None
			Desired heading of the Sphero.
		"""

		pass

	def stop(self):
		""" Method to stop the Sphero. """

		pass

	def init_heading(self):
		""" Method to set the initial heading angle for the Sphero. """

		pass

	def set_color(self, rgb):
		"""Method to set the color of the Sphero.

		Parameters
		----------
		rgb : tuple of int
			Tuple containing the RGB values of the desired color.
		"""

		pass

	def set_as_leader(self, channel=0):
		"""Method to set the Sphero as a leader.

		Parameters
		----------
		channel : int
			Channel you want to broadcast to.
		"""

		pass

	def set_as_follower(self, channel=0):
		"""Method to set the Sphero as a follower.

		Parameters
		----------
		channel : int
			Channel you want to listen to.
		"""

		pass

	def get_light_sensor_value(self):
		""" Method to get the light sensor value. """

		pass

	def calibrate_north(self):
		""" Method to calibrate the north on the Sphero """

		pass

	@property
	def heading(self):
		return self._heading

	@property
	def speed(self):
		return self._speed

	@heading.setter
	def heading(self, heading):
		self._heading = heading % 360

	@speed.setter
	def speed(self, speed=0):
		self._speed = max(0, min(speed, 255))
###################################################


###################################################
class SpheroMini(Sphero):
	"""The subclass of Sphero for a single Sphero Mini.

	Parameters
	----------
	tag_id : int
		The Apriltag ID of the Sphero Mini.
	verbosity : int
		The verbosity level of the instance.
	try_num : int
		Number of time to try connecting to the Sphero Mini.

	Attributes
	----------
	id : int
		The Apriltag ID of the Sphero Mini.
	verb : int
		The verbosity level of this class instance.
	connected : bool
		True if the Sphero Mini is connected.
	_heading : int
		The desired heading angle for the Sphero Mini.
	_speed : int
		The desired speed for the Sphero Mini.
	address : str
		The MAC address of the Sphero Mini.
	name : str
		The name of the Sphero Mini.
	"""

	def __init__(self, tag_id=0, verbosity=1, try_num=1):
		"""Constructor for SpheroMini class.

		Parameters
		----------
		tag_id : int
			The Apriltag ID of the Sphero Mini.
		verbosity : int
			The verbosity level of the instance.
		try_num : int
			Number of time to try connecting to the Sphero Mini.

		Yields
		------
		SpheroMini
			A SpheroMini instance.
		"""

		# Create an instance of Sphero
		super().__init__(tag_id, verbosity)

		# Add values for Sphero Mini
		self.address = MAC_ADDRESS[MINI][self.id]
		self.name = "Sphero Mini {0}".format(self.id)

		# Connect to the Sphero
		if self.verb == 0:
			print("\nConnecting to {0}".format(self.name))

		try_count = 1
		while not self.connected:
			if try_count <= try_num:
				try:
					if self.verb == 0:
						print("\rTry {0} of {1}".format(try_count, try_num), end="")
					self.sphero = sphero_mini(self.address, verbosity=verbosity)
					self.connected = True
				except BTLEDisconnectError:
					try_count += 1

			else:
				raise CantConnectError("Can't connect to {} after {} attempts.".format(self.name, try_count - 1))

		if self.verb == 0:
			print("\nConnected  to {0}\n".format(self.name))

		self.connected = True

	def disconnect(self):
		""" Method to put the Sphero Mini to sleep and disconnect it. """

		if self.connected:
			try:
				self.sphero.sleep()
				self.sphero.disconnect()
			except BTLEInternalError:
				print("The device is already disconnected.")

	def send(self, speed=None, heading=None):
		"""Method to send speed and heading command to the Sphero Mini.

		Parameters
		----------
		speed : int or None
			Desired speed of the Sphero Mini.
		heading : int or None
			Desired heading of the Sphero Mini.
		"""

		if speed is not None:
			self.speed = speed
		if heading is not None:
			self.heading = heading

		self.sphero.roll(speed=self.speed, heading=self.heading)

	def stop(self):
		""" Method to stop the Sphero Mini. """

		self.sphero.roll(speed=0, heading=self.heading)

	def init_heading(self):
		""" Method to set the initial heading angle for the Sphero Mini. """

		self.sphero.stabilization(False)
		self.sphero.roll(speed=0, heading=0)
		self.sphero.setBackLEDIntensity(brightness=255)

		print("Head the backLED toward the left of the camera frame.")
		input("Press Enter to continue...")

		self.sphero.resetHeading()
		self.sphero.setBackLEDIntensity(brightness=0)
		self.sphero.stabilization(True)

	def set_color(self, rgb):
		"""Method to set the color of the Sphero Mini.

		Parameters
		----------
		rgb : tuple of int
			Tuple containing the RGB values of the desired color.
		"""

		self.sphero.setLEDColor(red=rgb[0], green=rgb[1], blue=rgb[2])

	def set_as_leader(self, channel=0):
		"""Method to set the Sphero as a leader.

		Parameters
		----------
		channel : int
			Channel you want to broadcast to.
		"""

		print("This function is not available on Sphero Mini.")

	def set_as_follower(self, channel=0):
		"""Method to set the Sphero as a follower.

		Parameters
		----------
		channel : int
			Channel you want to listen to.
		"""

		print("This function is not available on Sphero Mini.")

	def get_light_sensor_value(self):
		""" Method to get the light sensor value. """

		print("This function is not available on Sphero Mini.")

	def calibrate_north(self):
		""" Method to calibrate the north on the Sphero """

		print("This function is not available on Sphero Mini.")
###################################################


###################################################
class SpheroBolt(Sphero):
	"""The subclass of Sphero for a single Sphero Bolt.

	Parameters
	----------
	tag_id : int
		The Apriltag ID of the Sphero Bolt.
	verbosity : int
		The verbosity level of the instance.
	try_num : int
		Number of time to try connecting to the Sphero Bolt.

	Attributes
	----------
	id : int
		The Apriltag ID of the Sphero Bolt.
	verb : int
		The verbosity level of this class instance.
	connected : bool
		True if the Sphero Bolt is connected.
	_heading : int
		The desired heading angle for the Sphero Bolt.
	_speed : int
		The desired speed for the Sphero Bolt.
	address : str
		The MAC address of the Sphero Bolt.
	name : str
		The name of the Sphero Bolt.
	toerr : int
		Number of time the Sphero encountered a TimeOut Error
	"""

	def __init__(self, tag_id=0, verbosity=1, try_num=1):
		"""Constructor for SpheroBolt class.

		Parameters
		----------
		tag_id : int
			The Apriltag ID of the Sphero Bolt.
		verbosity : int
			The verbosity level of the instance.
		try_num : int
			Number of time to try connecting to the Sphero Bolt.

		Yields
		------
		SpheroBolt
			A SpheroBolt instance.
		"""

		# Create an instance of Sphero
		super().__init__(tag_id, verbosity)

		# Add values for Sphero Bolt
		self.address = MAC_ADDRESS[BOLT][self.id]
		self.name = "Sphero Bolt {0}".format(self.id)

		# Connect to the Sphero
		if self.verb == 0:
			print("\nConnecting to {0}".format(self.name))

		try_count = 1
		while not self.connected:
			if try_count <= try_num:
				try:
					if self.verb == 0:
						print("\rTry {0} of {1}".format(try_count, try_num), end="")
					self.sphero = PySphero(mac_address=self.address)
					self.connected = True
				except BTLEDisconnectError:
					try_count += 1

			else:
				raise CantConnectError("Can't connect to {} after {} attempts.".format(self.name, try_count - 1))

		# Call __enter__ as required by the class (because not used with "with")
		self.sphero.__enter__()
		self.sphero.power.wake()

		if self.verb == 0:
			print("\nConnected to {0}\n".format(self.name))

		self.connected = True
		self.not_received = True
		self.resp = {}
		self.toerr = 0

	def disconnect(self):
		""" Method to put the Sphero Bolt to sleep and disconnect it. """

		if self.connected:
			try:
				if self.is_follower:
					self.sphero.sensor.request(command_id=CustomPySpheroCommand.stop_following,
											   target_id=0x12,)
				if self.is_leader:
					self.sphero.sensor.request(command_id=CustomPySpheroCommand.stop_IR_broadcast,
											   target_id=0x12, )
				self.sphero.power.enter_soft_sleep()
				self.sphero.sphero_core.close()
			except BTLEInternalError:
				print("The device is already disconnected.")

	def send(self, speed=None, heading=None):
		"""Method to send speed and heading command to the Sphero Bolt.

		Parameters
		----------
		speed : int or None
			Desired speed of the Sphero Bolt.
		heading : int or None
			Desired heading of the Sphero Bolt.
		"""

		if speed is not None:
			self.speed = speed
		if heading is not None:
			self.heading = heading

		try:
			self.sphero.driving.drive_with_heading(self.speed, self.heading)
		except PySpheroTimeoutError:
			self.toerr += 1

	def stop(self):
		""" Method to stop the Sphero Bolt. """

		self.sphero.driving.drive_with_heading(0, self.heading)

	def init_heading(self):
		""" Method to set the initial heading angle for the Sphero Bolt. """

		self.sphero.driving.drive_with_heading(0, 0)
		self.sphero.driving.set_stabilization(StabilizationIndex.no_control_system)
		self.sphero.user_io.set_all_leds_8_bit_mask(Color(), Color(0, 0, 255))

		print("Head the backLED toward the left of the camera frame.")
		input("Press Enter to continue...")

		self.sphero.driving.reset_yaw()
		self.sphero.user_io.set_all_leds_8_bit_mask(Color(), Color())
		self.sphero.driving.set_stabilization(StabilizationIndex.full_control_system)

	def set_color(self, rgb):
		"""Method to set the color of the Sphero Bolt.

		Parameters
		----------
		rgb : tuple of int
			Tuple containing the RGB values of the desired color.
		"""

		# c = Color(rgb[0], rgb[1], rgb[2])
		# self.sphero.user_io.set_all_leds_8_bit_mask(c, c)
		# self.sphero.user_io.set_led_matrix_one_color(c)
		pass

	def set_as_leader(self, channel=0):
		"""Method to set the Sphero as a leader.

		Parameters
		----------
		channel : int
			Channel you want to broadcast to.
		"""

		self.is_leader = True
		self.sphero.sensor.request(command_id=CustomPySpheroCommand.start_IR_broadcast,
								   target_id=0x12,
								   data=[2*channel, 2*channel+1])

	def set_as_follower(self, channel=0):
		"""Method to set the Sphero as a follower.

		Parameters
		----------
		channel : int
			Channel you want to listen to.
		"""

		self.is_follower = True
		self.sphero.sensor.request(command_id=CustomPySpheroCommand.start_following,
								   target_id=0x12,
								   data=[2*channel, 2*channel+1])

	def get_light_sensor_value(self):
		""" Method to get the light sensor value.

		Return
		------
		float
			The ligth sensor value in lux.
		"""

		value = self.sphero.sensor.get_ambient_light_sensor_value()
		return value

	def calibrate_north(self):
		""" Method to calibrate the north on the Sphero """

		self.sphero.sensor.magnetometer_calibrate_to_north()

	def notify_north(self):
		""" Method to notify the North? NOT WORKING, TO LOOK IN THE FUTURE. """

		self.sphero.sensor.notify(command_id=SensorCommand.magnetometer_north_yaw_notify, callback=self.notify_callback)

	def notify_callback(self, response: Packet):
		""" Notify callback. NOT WORKING, TO LOOK IN THE FUTURE.

		Parameters
		----------
		response : Packet
			The response from the Sphero
		"""

		bytes_list = response.data
		res = 0
		for b in bytes_list:
			res += res * 256 + int(b)

		print("{{}} : ".format(self.id) + str(bytes_list) + " -> {}".format(res))

		self.sphero.driving.drive_with_heading(speed=0, heading=res)

		self.sphero.driving.reset_yaw()

		self.sphero.driving.drive_with_heading(speed=0, heading=360-res)

	def get_sensor_values(self, sensor):
		""" Method to get the value of the given sensors.

		Parameters
		----------
		sensor : Quaternion, Attitude, Accelerometer, AccelOne,  Locator, Velocity, Speed, CoreTime, Gyroscope or AmbientLight
			list of the sensors.

		Return
		------
		dict
			A dictionary with all the sensor data.
		"""

		self.resp = {}
		self.not_received = True
		self.sphero.sensor.set_notify(self.sensors_callback, sensor, timeout=2)
		t = time()
		while self.not_received:
			if time() - t > 2:
				self.sphero.sensor.cancel_notify_sensors()
				break
			continue

		return self.resp

	def sensors_callback(self, response):
		self.sphero.sensor.cancel_notify_sensors()
		self.resp = response
		self.not_received = False
###################################################


###################################################
class MultiSphero:
	"""The super class of multiple Sphero.

	Parameters
	----------
	tag_ids : tuple of int
		The Apriltag IDs of the Spheros.
	verbosity : int
		The verbosity level of the instance.

	Attributes
	----------
	ids : tuple of int
		The Apriltag IDs of the Spheros.
	verb : int
		The verbosity level of this class instance.
	num : int
		The number of Sphero in this swarm.
	headings : ndarray
		The desired heading angles for all the Spheros.
	speeds : ndarray
		The desired speeds for all the Sphero.
	spheros : ndarray of Sphero
		A numpy array containing all the Sphero instances.
	_lf_channel : int
		The IR channel for the leader/followers method.
	"""

	def __init__(self, tag_ids=(), verbosity=0):
		"""Constructor for MultiSphero class.

		Parameters
		----------
		tag_ids : tuple of int
			The Apriltag IDs of the Spheros.
		verbosity : int
			The verbosity level of the instance.

		Yields
		------
		MultiSphero
			A MultiSphero instance.
		"""

		self.ids = tag_ids
		self.verb = verbosity
		self.num = len(tag_ids)
		self.headings = zeros(self.num)
		self.speeds = zeros(self.num)
		self.spheros = zeros(self.num, dtype=Sphero)
		self._lf_channel = 0

	def __del__(self):
		""" Destructor for MultiSphero class. """

		print("\r  \r", end='')
		for sphero in self.spheros:
			sphero.disconnect()

	def set_colors(self, ids=None, color=None):
		"""Method to set the colors for the Spheros

		If no ids are provided, all the Spheros will have the color of the
		first element on the color tuple

		Parameters
		----------
		ids : tuple of int or None
			The Apriltags IDs of the Spheros to change the color.
		color : tuple of int or None
			The color to change to in RGB values.
		"""

		# if type(color) is not tuple:
		# 	pass
		# elif len(color) is not 3:
		# 	pass
		#
		# if ids is None:
		# 	c = color
		# 	for sphero in self.spheros:
		# 		sphero.set_color(c)
		# else:
		# 	for (i, val) in enumerate(ids):
		# 		try:
		# 			ind = self.ids.index(val)
		# 			self.spheros[ind] = color
		# 		except ValueError:
		# 			print("Invalid id.")
		pass

	def set_headings(self, ids=None, heading=None):
		"""Method to set the headings for the Spheros

		If no ids are provided, all the Spheros will have the heading of the
		first element on the heading tuple

		Parameters
		----------
		ids : tuple of int or None
			The Apriltags IDs of the Spheros to change the headings.
		heading : tuple of int
			The headings to change to.
		"""

		if ids is None:
			if type(heading) is tuple:
				h = heading[0]
			else:
				h = heading
			self.headings = ones(self.num) * h
		else:
			for (i, val) in enumerate(ids):
				try:
					ind = self.ids.index(val)
					self.headings[ind] = heading[i] % 360
				except ValueError:
					print("Invalid id.")

		for (i, sphero) in enumerate(self.spheros):
			sphero.heading = self.headings[i]

	def set_speeds(self, ids=None, speed=None):
		"""Method to set the speeds for the Spheros

		If no ids are provided, all the Spheros will have the speeds of the
		first element on the speed tuple

		Parameters
		----------
		ids : tuple of int or None
			The Apriltags IDs of the Spheros to change the speeds.
		speed : tuple of int
			The speeds to change to.
		"""

		if ids is None:
			if type(speed) is tuple:
				s = speed[0]
			else:
				s = speed
			self.speeds = ones(self.num) * s
		else:
			for (i, val) in enumerate(ids):
				try:
					ind = self.ids.index(val)
					self.spheros[ind] = max(0, min(speed[i], 255))
				except ValueError:
					print("Invalid id.")

		for (i, sphero) in enumerate(self.spheros):
			sphero.speed = self.speeds[i]

	def send(self, ids=None):
		"""Method to send the heading and speed of specific Spheros.

		Parameters
		----------
		ids : tuple of int or None
			The Apriltags IDs of the Spheros to send the data.
		"""

		if ids is None:
			for (i, sphero) in enumerate(self.spheros):
				sphero.send()
		else:
			if type(ids) is int:
				self.spheros[ids].send()
			else:
				for i in ids:
					self.spheros[i].send()

	def stop(self, ids=None):
		"""Method to stop specific Spheros.

		Parameters
		----------
		ids : tuple of int or None
			The Apriltags IDs of the Spheros to stop.
		"""

		if ids is None:
			for (i, sphero) in enumerate(self.spheros):
				sphero.stop()
		else:
			if type(ids) is int:
				self.spheros[ids].stop()
			else:
				for i in ids:
					self.spheros[i].stop()

	def init_heading(self):
		""" Method to initialize the initial angle of multiple Spheros. """

		pass

	def setup_leader_followers(self, channel=0):
		""" Method to setup a leader/followers of multiple Spheros. """

		pass

	def calibrate_north(self):
		""" Method to calibrate north on the spheros """

		pass

	@property
	def lf_channel(self):
		return self._lf_channel

	@lf_channel.setter
	def lf_channel(self, channel):
		if type(channel) is not int:
			print("lf_channel should be an int.")
		else:
			self._lf_channel = min(max(channel, 0), 4)
###################################################


###################################################
class MultiSpheroMini(MultiSphero):
	"""The subclass of multiple Sphero Mini.

	Parameters
	----------
	tag_ids : tuple of int
		The Apriltag IDs of each Sphero Mini.
	verbosity : int
		The verbosity level of the instance.
	try_num : int
		Number of time to try connecting to each Sphero Mini.

	Attributes
	----------
	ids : tuple of int
		The Apriltag IDs of each Sphero Mini.
	verb : int
		The verbosity level of this class instance.
	num : int
		The number of Sphero Mini in this swarm.
	headings : ndarray
		The desired heading angles for all the Sphero Mini.
	speeds : ndarray
		The desired speeds for all the Sphero Mini.
	spheros : ndarray of SpheroMini
		A numpy array containing all the Sphero Mini instances.
	"""

	def __init__(self, tag_ids=(), verbosity=0, try_num=1):
		"""Constructor for MultiSpheroMini class.

		Parameters
		----------
		tag_ids : tuple of int
			The Apriltag IDs of all the Sphero Mini.
		verbosity : int
			The verbosity level of the instance.
		try_num : int
			Number of time to try connecting to all the Sphero Mini.

		Yields
		------
		MultiSpheroMini
			A MultiSpheroMini instance.
		"""

		super().__init__(tag_ids=tag_ids, verbosity=verbosity)

		self.spheros = zeros(self.num, dtype=SpheroMini)

		if self.verb == 0:
			print("\n[Connecting to Swarm]")

		for (i, ids) in enumerate(self.ids):
			if self.verb == 0:
				print("\rConnecting to ID {0} ({1}/{2})...".format(ids, i + 1, self.num), end='')

			try:
				self.spheros[i] = SpheroMini(tag_id=ids, verbosity=-1, try_num=try_num)
			except CantConnectError:
				print("\nCan't connect to Sphero {0}. Exiting...".format(ids))
				exit()

		if self.verb == 0:
			print("\n[Connected to Swarm]\n")

	def init_heading(self):
		""" Method to initialize the initial angle of multiple Spheros. """

		for s in self.spheros:
			s.sphero.stabilization(False)
			s.sphero.roll(speed=0, heading=0)
			s.sphero.setBackLEDIntensity(brightness=255)

		print("*** Head the backLED toward the left of the camera frame. ***")
		input("Press Enter to continue...")

		print("\nLoop started\n")

		for s in self.spheros:
			s.sphero.resetHeading()
			s.sphero.setBackLEDIntensity(brightness=0)
			s.sphero.stabilization(True)

	def setup_leader_followers(self, channel=0):
		""" Method to setup a leader/followers of multiple Spheros. """

		print("This function is not available with Sphero Mini.")

	def calibrate_north(self):
		""" Method to calibrate north on the spheros """

		print("This function is not available with Sphero Mini.")
###################################################


###################################################
class MultiSpheroBolt(MultiSphero):
	"""The subclass of multiple Sphero Bolt.

	Parameters
	----------
	tag_ids : tuple of int
		The Apriltag IDs of each Sphero Bolt.
	verbosity : int
		The verbosity level of the instance.
	try_num : int
		Number of time to try connecting to each Sphero Bolt.

	Attributes
	----------
	ids : tuple of int
		The Apriltag IDs of each Sphero Bolt.
	verb : int
		The verbosity level of this class instance.
	num : int
		The number of Sphero Bolt in this swarm.
	headings : ndarray
		The desired heading angles for all the Sphero Bolt.
	speeds : ndarray
		The desired speeds for all the Sphero Bolt.
	spheros : ndarray of SpheroBolt
		A numpy array containing all the Sphero Bolt instances.
	"""

	def __init__(self, tag_ids=(), verbosity=0, try_num=1):
		"""Constructor for MultiSpheroBolt class.

		Parameters
		----------
		tag_ids : tuple of int
			The Apriltag IDs of all the Sphero Bolt.
		verbosity : int
			The verbosity level of the instance.
		try_num : int
			Number of time to try connecting to all the Sphero Bolt.

		Yields
		------
		MultiSpheroBolt
			A MultiSpheroBolt instance.
		"""

		super().__init__(tag_ids=tag_ids, verbosity=verbosity)

		self.spheros = zeros(self.num, dtype=SpheroBolt)

		if self.verb == 0:
			print("\n[Connecting to Swarm]")

		for (i, ids) in enumerate(self.ids):
			if self.verb == 0:
				print("\rConnecting to ID {0} ({1}/{2})...".format(ids, i + 1, self.num), end='')

			try:
				self.spheros[i] = SpheroBolt(tag_id=ids, verbosity=-1, try_num=try_num)
			except CantConnectError:
				print("\nCan't connect to Sphero {0}. Exiting...".format(ids))
				exit()

		if self.verb == 0:
			print("\n[Connected to Swarm]\n")

	def init_heading(self):
		""" Method to initialize the initial angle of multiple Spheros. """

		for s in self.spheros:
			s.sphero.driving.drive_with_heading(0, 0)
			s.sphero.driving.set_stabilization(StabilizationIndex.no_control_system)
			s.sphero.user_io.set_all_leds_8_bit_mask(Color(), Color(0, 0, 255))

		print("*** Head the backLED toward the left of the camera frame. ***")
		input("Press Enter to continue...")

		print("\nLoop started\n")

		for s in self.spheros:
			s.sphero.driving.reset_yaw()
			s.sphero.user_io.set_all_leds_8_bit_mask(Color(), Color())
			s.sphero.driving.set_stabilization(StabilizationIndex.full_control_system)

	def setup_leader_followers(self, channel=0):
		""" Method to setup a leader/followers of multiple Spheros. """

		if type(channel) is not int:
			print("The channel should be an int.")
		else:
			self.lf_channel = channel

			for (i, s) in enumerate(self.spheros):
				if i == 0:
					s.set_as_leader(channel=self.lf_channel)
				else:
					s.set_as_follower(channel=self.lf_channel)

	def calibrate_north(self):
		""" Method to calibrate north on the spheros """

		for (i, s) in enumerate(self.spheros):
			s.calibrate_north()

	def notify_north(self):
		""" Method to notify the North? """

		for (i, s) in enumerate(self.spheros):
			s.notify_north()
###################################################
