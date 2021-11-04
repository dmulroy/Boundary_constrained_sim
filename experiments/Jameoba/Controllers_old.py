"""
This module contains many controllers for Sphero(s) Mini/Bolt to be used
with ROS.
"""

###################################################
from numpy import zeros, ndarray
import numpy as np
from math import atan2, sqrt, pi
from time import time
import datetime
import rospy
from std_msgs.msg import String
from pid_controller.pid import PID
from .Spheros import SpheroMini, SpheroBolt, \
	MultiSpheroMini, MultiSpheroBolt
from scipy import stats
from scipy.interpolate import RegularGridInterpolator
import statistics
from scipy.optimize import curve_fit


###################################################


###################################################
# ------------------- Classes ------------------- #
###################################################


###################################################

###################################################


###################################################
class SpheroController:
	"""PID controller of a single Sphero.

	Parameters
	----------
	single_sphero : SpheroMini or SpheroBolt or int
		The Sphero instance to be controlled. If an int,
		this is the controller for a simulation.
	kp : float, optional
		Proportional gain of the PID.
	ki : float, optional
		Integral gain of the PID.
	kd : float, optional
		Derivative gain of the PID.
	is_alone : bool, optional
		Set to False if the controller is used in a swarm (multiple controller).

	Attributes
	----------
	sphero : SpheroMini or SpheroBolt or None
		The Sphero instance that is controlled.
	at_id : int
		The Apriltag ID of the Sphero.
	x : int
		Current position in x.
	y : int
		Current position in y.
	tar_x : int
		Target position in x.
	tar_y : int
		Target position in y.
	_ulim : int
		Ultimate speed limit for the Sphero.
	_freq : float
		Maximum frequency to send command to the Sphero via Bluetooth.
	speed_ctl : PID
		The speed PID controller.
	last : float
		The time at which data was last sent to the Sphero via Bluetooth.
	"""

	def __init__(self, single_sphero, kp=1., ki=0., kd=0., is_alone=True, method='pot'):
		"""Constructor of the SpheroController class.

		Parameters
		----------
		single_sphero : SpheroMini or SpheroBolt or int
			The Sphero instance to be controlled. If an int,
			this is the controller for a simulation.
		kp : float, optional
			Proportional gain of the PID.
		ki : float, optional
			Integral gain of the PID.
		kd : float, optional
			Derivative gain of the PID.
		is_alone : bool, optional
			Set to False if the controller is used in a swarm (multiple controller).

		Yields
		------
		SpheroController
			A PID controller for a single Sphero Mini/Bolt.
		"""

		self.x = self.y = self.tar_x = self.tar_y = 0
		self._ulim = 255
		self._freq = 2.
		# Check if this is a simulated Sphero.
		if type(single_sphero) is int:
			self.sphero = None
			self.at_id = single_sphero
		else:
			self.sphero = single_sphero
			self.at_id = single_sphero.id

		# Initialize the PID speed controller.
		self.speed_ctl = PID(p=kp, i=ki, d=kd)
		self.speed_ctl.target = 0
		# Set the last data sending time as now.
		self.last = time()
		# If this is not a part of a swarm, start ROS
		if is_alone:
			rospy.init_node("Sphero_" + str(self.at_id), anonymous=False)
			rospy.Subscriber("target", String, self.ros_set_target, queue_size=2)
			rospy.Subscriber("state", String, self.ros_callback, queue_size=2)
			rospy.spin()
		self.control_method = method
		self.A = kp
		self.alpha = ki
		self.beta = kd
		self.t_k_1 = 0
		#self.df = pd.DataFrame(columns=['bot', 'time', 'x', 'y', 'vx', 'vy'])
		#self.file_name='data' + str(datetime.datetime.now())

	class calc_velocity:

		def __init__(self, p1, p2, R, A, alpha, beta):
			self.A = A
			self.alpha = alpha
			self.beta = beta

			self.p1 = p1  # x of the target
			self.p2 = p2  # y of the target

			self.R = R  # radius around the target

			self.vmax = 255
			self.vmin = 0

			self.b = 20
			self.res = .1

			self.vxrange = np.linspace(self.vmin, self.vmax, 20)
			self.vyrange = np.linspace(self.vmin, self.vmax, 20)

			self.xmin = self.p1 - self.b
			self.xmax = self.p1 + self.b

			self.ymin = self.p2 - self.b
			self.ymax = self.p2 + self.b

			self.xcount = int(round((self.xmax - self.xmin) / self.res))
			self.ycount = int(round((self.ymax - self.ymin) / self.res))

			self.xp = np.linspace(self.xmin, self.xmax, self.xcount)
			self.yp = np.linspace(self.ymin, self.ymax, self.ycount)
			self.xx, self.yy = np.meshgrid(self.xp, self.yp)

			self.d = np.sqrt((1 / self.R ** 2) * ((self.xx - self.p1) ** 2 + (self.yy - self.p2) ** 2))
			self.d1 = -2 * self.p1 + 2 * self.xx
			self.d2 = -2 * self.p2 + 2 * self.yy
			self.f2x = self.A * (2 * (2 / self.R ** 2) * self.d1 * self.d * (np.log(self.d)) ** 2 + (
					2 / self.R ** 2) * self.d1 * self.d * (np.log(self.d)))
			self.f2y = self.A * (2 * (2 / self.R ** 2) * self.d2 * self.d * (np.log(self.d)) ** 2 + (
					2 / self.R ** 2) * self.d2 * self.d * (np.log(self.d)))
			self.fminx = np.min(self.f2x)
			self.fmaxx = np.max(self.f2x)
			self.fminy = np.min(self.f2y)
			self.fmaxy = np.max(self.f2y)
			self.fyrange = np.linspace(0, self.fmaxy, 20)
			self.fxrange = np.linspace(0, self.fmaxx, 20)
			self.slopex, self.interceptx, self.r_value, self.p_value, self.td_err = stats.linregress(self.fxrange,
																									 self.vxrange)
			self.slopey, self.intercepty, self.r_value, self.p_value, self.td_err = stats.linregress(self.fyrange,
																									 self.vyrange)
			self.slopem, self.interceptm, self.r_value, self.p_value, self.td_err = stats.linregress(
				np.linspace(0, np.max((self.f2y ** 2 + self.f2x ** 2) ** 0.5), 20),
				np.linspace(0, self.vmax, 20))

		def out_force(self, x, y, vx, vy):
			d = np.sqrt((1 / self.R ** 2) * ((x - self.p1) ** 2 + (y - self.p2) ** 2))
			d1 = -2 * self.p1 + 2 * x
			d2 = -2 * self.p2 + 2 * y
			f2x = self.A * (
					2 * (2 / self.R ** 2) * d1 * d * (np.log(d)) ** 2 + (2 / self.R ** 2) * d1 * d * (np.log(d)))
			f2y = self.A * (
					2 * (2 / self.R ** 2) * d2 * d * (np.log(d)) ** 2 + (2 / self.R ** 2) * d2 * d * (np.log(d)))
			VX = self.slopem * f2x + self.interceptm
			VY = self.slopem * f2y + self.interceptm
			Vx = -self.alpha * VX - self.beta * vx
			Vy = -self.alpha * VY - self.beta * vy

			return Vx, Vy

	class controller:
		def __init__(self, px, py, R, limits, vmax):
			self.px = px  # center x
			self.py = py  # center y
			self.t1 = 0.5
			self.R = R  # radius of zero contour
			self.vmax = vmax
			self.alpha = self.vmax  # max velocity
			self.beta = 0  # dampning term
			self.limits = limits
			self.b = 6  # how far left or right
			self.res = .04  # resolution
			self.A = 1  # constant
			# Settting up the grid of the potential field
			self.xmin = self.px - self.b
			self.xmax = self.px + self.b
			self.ymin = self.py - self.b
			self.ymax = self.py + self.b
			self.xcount = int(round(self.xmax - self.xmin / self.res))  # find out how many numbers in the x
			self.ycount = int(round(self.ymax - self.ymin / self.res))  # find out how many numbers in the y
			self.xp = np.linspace(self.xmin, self.xmax, self.xcount)  # set up range for x
			self.yp = np.linspace(self.ymin, self.ymax, self.ycount)  # set up range for y
			self.xx, self.yy = np.meshgrid(self.xp, self.yp)  # create mesh grid
			self.d = np.sqrt((1 / self.R ** 2) * ((self.xx - self.px) ** 2 + (self.yy - self.py) ** 2))  # create d
			self.f = self.d ** 2 * np.log(self.d)  # potential function
			self.maxf = np.max(self.f)  # find max value in potential field
			# if we want to limit the field
			if self.limits == True:
				self.f = self.f / self.maxf  # scale down potential field
			# create gradient
			self.d1 = -2 * self.px + 2 * self.xx
			self.d2 = -2 * self.py + 2 * self.yy
			# self.fx=np.nan_to_num(self.A*(2*(2/self.R**2)*self.d1*self.d*(np.log(self.d))**2 + (2/self.R**2)*self.d1*self.d*(np.log(self.d)))) # gradient x
			# self.fy=np.nan_to_num(self.A*(2*(2/self.R**2)*self.d2*self.d*(np.log(self.d))**2 + (2/self.R**2)*self.d2*self.d*(np.log(self.d))))  # gradient y
			self.fx = np.nan_to_num(self.A * (2 * (2 / self.R ** 2) * self.d1 * self.d * (np.log(self.d ** 2)) + (
						2 / self.R ** 2) * self.d1 * self.d * (np.log(self.d))))  # gradient x
			self.fy = np.nan_to_num(self.A * (2 * (2 / self.R ** 2) * self.d2 * self.d * (np.log(self.d ** 2)) + (
						2 / self.R ** 2) * self.d2 * self.d * (np.log(self.d))))  # gradient y
			self.A = np.max(self.fx)
			if self.limits == True:
				self.fx = self.fx / self.A
				self.fy = self.fy / self.A
			# Create a interpolated version for us to easily access
			self.fny = RegularGridInterpolator((self.xp, self.yp), self.fy)  # gradient of y
			self.fnx = RegularGridInterpolator((self.xp, self.yp), self.fx)  # gradient of x

		##################################################################################
		# Based on the potential field this is a function that determines the magnitude and direction the robot should travel in
		def out_vel(self, x, y, vx, vy):
			VY = self.fny((y, x))
			VX = self.fnx((y, x))
			Vx = -self.alpha * VX - self.beta * vx
			Vy = -self.alpha * VY - self.beta * vy
			V = np.sqrt(Vx ** 2 + Vy ** 2)
			if V > self.vmax:
				V = 255
			else:
				V = V
			theta = (180 * np.arctan2(Vy, Vx) / np.pi) % 360
			return (V, theta)

		# Based on the potential field this is a function that determines the magnitude and direction the robot should travel in
		def out_vel2(self, x, y, vx, vy):
			VY = self.fny((y, x))
			VX = self.fnx((y, x))
			mag = np.sqrt(VX ** 2 + VY ** 2)
			VX = VX / mag
			VY = VY / mag
			Vx = self.alpha * VX - self.beta * vx
			Vy = self.alpha * VY - self.beta * vy
			V = np.sqrt(Vx ** 2 + Vy ** 2)
			if V.any() > self.vmax:
				V = 255
			else:
				V = V
			theta = (180 * np.arctan2(Vy, Vx) / np.pi) % 360
			return (V, theta)

		def out_vel3(self, x, y, vx, vy):
			VY = self.fny((y, x))
			VX = self.fnx((y, x))
			mag = np.sqrt((VX ** 2) + (VY ** 2))
			Vx = VX / mag
			Vy = VY / mag
			Vxx = -self.alpha * Vx - self.beta * vx
			Vyy = -self.alpha * Vy - self.beta * vy
			return (Vxx, Vyy)

		def out_vel4(self, x, y, vx, vy, time):
			VY = self.fny((y, x))
			VX = self.fnx((y, x))
			g = 4
			theta = 180
			if self.t1 > time and np.cos(theta) * abs((x - self.px)) < abs(2 * self.R) and np.sin(theta) * abs(
					(y - self.py)) < abs(2 * self.R):
				VY = self.fny((y, x))
				VX = self.fnx((y, x))
				mag = np.sqrt(VX ** 2 + VY ** 2)
				VX = VX / mag
				VY = VY / mag
				Vx = self.alpha * VX - self.beta * vx
				Vy = self.alpha * VY - self.beta * vy
				if Vx >= 0:
					Vx = Vx - g * Vx
				else:
					Vx = Vx + g * Vx
				if Vy >= 0:
					Vy = Vy - g * Vy
				else:
					Vy = Vy + g * Vy
				V = Vx
				if V.any() > self.vmax:
					V = 255
				else:
					V = V
				theta = 180
			else:
				Vx = self.alpha * VX - self.beta * vx
				Vy = self.alpha * VY - self.beta * vy
				V = np.sqrt(Vx ** 2 + Vy ** 2)
				if V.any() > self.vmax:
					V = 255
				else:
					V = V
				theta = (180 * np.arctan2(Vy, Vx) / np.pi) % 360
			return (V, theta)

		def out_vel5(self, x, y, vx, vy, X, Y, diameter):
			K = 1
			Q = 2 * diameter
			VY = self.fny((y, x))
			VX = self.fnx((y, x))
			mag = np.sqrt(VX ** 2 + VY ** 2)
			fx = []
			fy = []
			for i in range(len(X)):
				print(i)
				dx = (x - X[i])
				dy = (y - Y[i])
				d = np.sqrt(dx ** 2 + dy ** 2)
				d32 = d ** (-1.5)
				dq = (1 / d) - (1 / Q)
				# print("d32= ",1/d32)
				Ux = np.nan_to_num(-K * (dx) * d32 * dq)
				Uy = np.nan_to_num(-K * (dy) * d32 * dq)
				print(Ux, Uy)
				# print(d)
				if d < 2 * diameter:
					fx.append(Ux)
					fy.append(Uy)
				else:
					fx.append(0)
					fy.append(0)
			print(fx, fy)
			vxr = np.sum(fx)
			vyr = np.sum(fy)
			print(vxr, vyr)
			magr = np.sqrt(vxr ** 2 + vyr ** 2)
			if magr == 0:
				Vxx = (VX / mag)
				Vyy = (VY / mag)
			else:
				Vxx = (VX / mag) + (vxr / magr)
				Vyy = (VY / mag) + (vyr / magr)
			Vx = -self.alpha * Vxx - self.beta * vx
			Vy = -self.alpha * Vyy - self.beta * vy
			V = np.sqrt(Vx ** 2 + Vy ** 2)
			if V > self.vmax:
				V = 255
			else:
				V = V
			theta = (180 * np.arctan2(Vy, Vx) / np.pi) % 360
			return (V, theta)

	def get_error_length(self):
		"""Method to get the length between the current position and the target position.

		Returns
		-------
		float
			The length between the current position and the target position.
		"""

		dx = self.tar_x - self.x
		dy = self.tar_y - self.y
		return sqrt(dx ** 2 + dy ** 2)

	def get_error_angle(self):
		"""Method to get the orientation of the target relative to the current position.

		Returns
		-------
		float
			The orientation of the target relative to the current position.
		"""

		dx = self.tar_x - self.x
		dy = self.tar_y - self.y
		ang = 180 * atan2(dy, dx) / pi
		return ang % 360

	def ros_set_target(self, data):
		"""Method used to update the closed loop target positions. Used with a ROS Subscriber.

		Parameters
		----------
		data : String
			Message published on the /target topic containing the position of the target tag.
		"""

		data_dict = eval(data.data)

		self.tar_x = data_dict[str(self.at_id)][0]
		self.tar_y = data_dict[str(self.at_id)][1]



	def ros_callback(self, data):
		"""Method used to run an iteration of the control loop. Used with a ROS Subscriber.

		Parameters
		----------
		data : String
			Message published on the /state topic containing the position of all tags.
		"""

		data_dict = eval(data.data)

		vx = (data_dict[str(self.at_id)][0] - self.x) / (time() - self.t_k_1)
		vy = (data_dict[str(self.at_id)][1] - self.y) / (time() - self.t_k_1)

		# Set the current positions
		self.x = data_dict[str(self.at_id)][0]
		self.y = data_dict[str(self.at_id)][1]
		self.t_k_1 = time()
		X = []
		Y = []
		for idx, i in data_dict.items():
			X = X.append(data_dict[str(idx)][0])
			Y = Y.append(data_dict[str(idx)][1])
		# Potential field controller
		################################################################################################################
		p1 = self.tar_x
		p2 = self.tar_y

		# First verion pot field
		R = 1
		# PotField = self.calc_velocity(p1, p2, R, self.A, self.alpha, self.beta)
		# ` Vx, Vy = PotField.out_force(self.x, self.y, vx, vy)
		# mag = np.sqrt(Vx ** 2 + Vy ** 2)
		# nf2x = Vx / mag
		# nf2y = Vy / mag
		# ang = 180 * atan2(nf2y, nf2x) / pi
		# mag = 255`

		# Second version pot field
		PotField = self.controller(p1, p2, R=R, limits=True, vmax=255)
		mag, ang = PotField.out_vel5(self.x, self.y, vx, vy, X, Y, 20)

		if self.control_method == 'pot':
			head = int(ang % 360)
			speed = max(min(int(mag), self.ulim), 0)
		elif self.control_method == 'pid':
			head = int(self.get_error_angle())
			speed = max(min(int(self.speed_ctl(feedback=self.get_error_length())), self.ulim), 0)


		print("\n Bot#", self.at_id, 'mehtod:', self.control_method, "dist:",
			  np.sqrt((self.x - self.tar_x) ** 2 + (self.y - self.tar_y) ** 2),
			  "velocity", vx, vy, 'tk-1:', self.t_k_1, "speed_pot:", speed, "ang_pot:", int(ang % 360),
			  "speed_pid:", max(min(int(self.speed_ctl(feedback=self.get_error_length())), self.ulim), 0), "ang_pid:",
			  int(self.get_error_angle()), "ang_diff:", ang - int(self.get_error_angle()))

		#df2 = pd.DataFrame({"bot": [self.at_id], 'time': [self.t_k_1], 'x': [self.x], 'y': [self.y], 'vx': [vx], "vy": [vy]})
		#df2.to_csv("zzz.csv", mode='a', header=False)

		# Send it to the sphero
		freq = 1.0 / (time() - self.last)
		if freq <= self.freq:
			self.last = time()

			if self.sphero is None:
				print("\r[{0}] ----> {1:3d}° at {2} % speed.".format(self.at_id, head, int(100 * speed / 255)))
			else:
				self.sphero.send(speed=speed, heading=head)

	@property
	def ulim(self):
		return self._ulim

	@ulim.setter
	def ulim(self, lim):
		self._ulim = max(0, min(lim, 255))

	@property
	def freq(self):
		return self._freq

	@freq.setter
	def freq(self, f):
		self._freq = max(0., min(f, 20.))


###################################################


###################################################
class MultiSpheroController:
	"""PID controller of a multiple Spheros.

	Parameters
	----------
	multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
		The Spheros swarm instance to be controlled. If a tuple,
		this is the controllers for a simulation.
	kp : float, optional
		Proportional gain of the PID.
	ki : float, optional
		Integral gain of the PID.
	kd : float, optional
		Derivative gain of the PID.

	Attributes
	----------
	spheros : ndarray of SpheroMini or ndarray of SpheroBolt or tuple of int
		An numpy array of the SpheroMini/SpheroBolt or of the Apriltag IDs if this is a simulation.
	num : int
		Number of Sphero in the swarm.
	ids : tuple of int
		A tuple containing the Apriltag ID of the Spheros.
	swarm : MultiSpheroMini or MultiSpheroBolt or None
		The Sphero swarm instance or None if this is a simulation.
	controllers : ndarray of SpheroController
		An numpy array of the SpheroController.
	last : float
		The time at which data was last received from the /state topic.
	last_tar : float
		The time at which data was last received from the /target topic.
	freq_tar : float
		Frequency at which the /target topic receives messages.
	"""

	def __init__(self, multi_sphero, kp=1., ki=0., kd=0., method='pot'):
		"""Constructor of the SpheroController class.

		Parameters
		----------
		multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
			The Spheros swarm instance to be controlled. If a tuple,
			this is the controllers for a simulation.
		kp : float, optional
			Proportional gain of the PID.
		ki : float, optional
			Integral gain of the PID.
		kd : float, optional
			Derivative gain of the PID.

		Yields
		------
		MultiSpheroController
			A PID controller for multiple Sphero Mini/Bolt.
		"""

		if type(multi_sphero) is tuple:
			self.spheros = self.ids = multi_sphero
			self.num = len(multi_sphero)
			self.swarm = None
		else:
			self.swarm = multi_sphero
			self.spheros = self.swarm.spheros
			self.num = self.swarm.num
			self.ids = self.swarm.ids

		self.controllers = zeros(self.num, dtype=SpheroController)
		for i in range(self.num):
			self.controllers[i] = SpheroController(self.spheros[i], kp, ki, kd, is_alone=False, method=method)

		self.last = self.last_tar = time()
		self.freq_tar = 0
		print("Frequencies :\n| Apriltags |  Target  | Timeout Error |")

		rospy.init_node("Spheros" + str(self.ids[0]), anonymous=False)
		rospy.Subscriber("target", String, self.ros_set_target, queue_size=2)
		rospy.Subscriber("state", String, self.ros_callback, queue_size=2)
		rospy.spin()

	def __del__(self):
		""" Destructor of the SpheroController class. """

		print("\b\b  \n")

	def ros_set_target(self, data):
		"""Method used to update the closed loop target positions. Used with a ROS Subscriber.

		Parameters
		----------
		data : String
			Message published on the /target topic containing the position of the target tag.
		"""

		self.freq_tar = 1. / (time() - self.last_tar)
		self.last_tar = time()
		for ctl in self.controllers:
			ctl.ros_set_target(data)

	def ros_callback(self, data):
		"""Method used to run an iteration of the control loop. Used with a ROS Subscriber.

		Parameters
		----------
		data : String
			Message published on the /state topic containing the position of all tags.
		"""

		freq = 1. / (time() - self.last)
		self.last = time()
		print(
			"\r| {0:2.2f} Hz  | {1:2.2f} Hz | {2} Errors    |      ".format(freq, self.freq_tar, self.get_sum_error()),
			end='')
		for ctl in self.controllers:
			ctl.ros_callback(data)

	def get_sum_error(self):
		""" Method the total number of"""

		if type(self.spheros[0]) is SpheroMini:
			return "N/A"
		else:
			val = 0
			for s in self.spheros:
				val += s.toerr

			return val
###################################################
