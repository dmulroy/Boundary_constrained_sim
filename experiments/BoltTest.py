import time

from pysphero.core import Sphero
from pysphero.sensor import SensorCommand, Attitude, Accelerometer
from pysphero.packet import Packet
from pysphero.driving import StabilizationIndex
from enum import Enum
from Jameoba import MAC_ADDRESS, BOLT


class SpheroBoltTest:

	def __init__(self):
		self.res = 0
		self.current = 0
		self.tag_id = 0
		self.sphero = Sphero(mac_address=MAC_ADDRESS[BOLT][self.tag_id])
		self.sphero.__enter__()

		self.main()

	def cb(self, response):
		self.current = response[Attitude.yaw]
		print("Current : {0:.2f} deg".format(self.current))
		self.sphero.sensor.cancel_notify_sensors()

	def cb2(self, response: Packet):
		bytes_list = response.data
		for b in bytes_list:
			self.res += self.res * 256 + int(b)

		print("Notify north : {0:.2f} deg".format(self.res))

	# def cb3(self, response):
	# 	self.current = response[Accelerometer.x]
	# 	print("AccX : ", self.current)
	# 	self.sphero.sensor.cancel_notify_sensors()

	def main(self):

		self.sphero.power.wake()

		self.sphero.sensor.magnetometer_calibrate_to_north()

		input("Press Enter...")

		self.sphero.sensor.notify(command_id=SensorCommand.magnetometer_north_yaw_notify, callback=self.cb2)

		input("Press Enter...")

		self.sphero.driving.set_stabilization(StabilizationIndex.no_control_system)

		self.sphero.sensor.set_notify(self.cb, Attitude)

		input("Press Enter...")

		self.sphero.driving.drive_with_heading(speed=0, heading=int(self.current-self.res)%360)

		# input("Press Enter...")
		#
		# self.sphero.driving.drive_with_heading(speed=40, heading=90)
		#
		# while True:
		# 	self.sphero.sensor.set_notify(self.cb3, Accelerometer)
		#
		# input("Press Enter...")

		self.sphero.power.enter_soft_sleep()


class Command(Enum):
	start_broadcast = 0x27
	start_evading = 0x33
	start_following = 0x28
	stop_broadcast = 0x29
	stop_evading = 0x34
	stop_following = 0x32


if __name__ == "__main__":
	s = SpheroBoltTest()

