"""
This module contains the classes to track Apriltags to get the position
of multiple Sphero Mini/Bolt and the position of the target. This module
is to be used with ROS.
"""

###################################################
import os
import csv
import cv2
import datetime
from numpy import ndarray, array, ones, transpose, dot, \
	identity, roll, zeros, sqrt as npsqrt
from numpy.linalg import norm
from math import pi, cos, sin, atan2, sqrt
import apriltags3py.apriltags3 as at
import rospy
from rospy import Publisher
from std_msgs.msg import String
from time import time
import screeninfo
import numpy as np
###################################################


###################################################
# ------------------ Constants ------------------ #
###################################################


###################################################
CAM_FRONT = 4
CAM_OVER = 1
CAM_BRIO = 2

CAMERA_PARAMS = {CAM_FRONT: (556.5, 557.8, 316.2, 222.1),
                 CAM_OVER: (809.2, 803.5, 367.7, 238.2),
                 CAM_BRIO: (1118.3, 1112.9, 991.2, 548.7)}

K = {CAM_OVER: array([[1377.02, 0., 593.42],
                      [0., 1380.58, 454.02],
                      [0., 0., 1.]]),
     CAM_BRIO: array([[1118.93, 0., 992.82],
                      [0., 1113.48, 545.99],
                      [0., 0., 1.]])}

D = {CAM_OVER: array([4.57e-02, -3.73e-01, 2.42e-02, -5.05e-03, 3.06e-01]),
     CAM_BRIO: array([2.48e-01, -7.49e-01, 5.96e-03, 7.79e-03, 5.19e-01])}

CROP = {CAM_OVER: (0, 50, 1080, 1800),
        CAM_BRIO: (0, 500, 720, 1200)}

RES_480P = 0
RES_720P = 1
RES_1080P = 2
RES_1440P = 3
RES_4K = 4

RESOLUTION = {RES_480P: (640, 480),
              RES_720P: (1280, 720),
              RES_1080P: (1920, 1080),
              RES_1440P: (2560, 1440),
              RES_4K: (3840, 2160)}

TAG_SMALL = 0
TAG_BIG = 1

TAG_SIZE = {TAG_SMALL: 0.03,
            TAG_BIG: 0.05}
###################################################


###################################################
# ------------------- Classes ------------------- #
###################################################


###################################################
class AprilTagTracker:
	"""Class that use Apriltags to track tags positions.

	This class use Apriltags to track robots and publish
	their positions on the /state ROS topic. It also display
	and save the video. The tracking data can also be drew
	over the video.

	Parameters
	----------
	num : int
		Number of tags to track.
	cam_choice : int, optional
		The choice of camera that is used.
	res_choice : int, optional
		The resolution to use.
	tag_choice : int, optional
		The tags to use.
	trail_length : int, optional
		The length of the trail behind the tags in number of length.
	save_data : bool, optional
		Save data or not.
	save_video : bool, optional
		Save the video or not.
	crop_image : bool, optional
		Crop the image or not.
	testname : str or None, optional
		Name of the current test.
	unwarped : bool, optional
		Unwarp the video or not.

	Attributes
	----------
	camera_index : int
		The computer camera index.
	origin : tuple of int
		Origin of the tracking. Automatically set to the initial mean position.
	state : dict
		Message sent on the /state ROS topic.
	number_of_tags : int
		Number of tags to track.
	cam_choice : int
		The choice of camera.
	res_choice : int
		The choice of resolution.
	tag_choice : int
		The choice of tags.
	trail_length : int
		The length of the trail behind the tags in number of length.
	save_data : bool
		Save data or not.
	save_video : bool
		Save video or not.
	crop_image : bool
		Crop image or not.
	unwarped : bool
		Unwarp the video or not.
	data_dir : str
		Name of the directory in which to store the data.
	filename : str
		Data filename.
	video_name : str
		Video filename.
	video_no_overlay_name : str
		Video with no overlay filename.
	pt : ndarray
		Numpy array which contains the pixel coordinate of the tags.
	ptmean : ndarray
		Numpy array which contains the pixel coordinate of the mean of the tags.
	arrow_vec : ndarray
		Numpy array which contains the pixel coordinate af the direction arrow of each tags.
	crop_params : tuple of int
		Parameters used while cropping the video.
	pub : Publisher
		The publisher used to publish on the /state ROS topic.
	"""

	def __init__(self, num, cam_choice=CAM_BRIO, res_choice=RES_1080P, tag_choice=TAG_BIG, trail_length=2,
	             save_data=False, save_video=False, crop_image=False, testname=None, unwarped=False):
		"""Constructor of the SpheroController class.

		Parameters
		----------
		num : int
			Number of tags to track.
		cam_choice : int, optional
			The choice of camera that is used.
		res_choice : int, optional
			The resolution to use.
		tag_choice : int, optional
			The tags to use.
		trail_length : int, optional
			The length of the trail behind the tags in number of length.
		save_data : bool, optional
			Save data or not.
		save_video : bool, optional
			Save the video or not.
		crop_image : bool, optional
			Crop the image or not.
		testname : str or None, optional
			Name of the current test.
		unwarped : bool, optional
			Unwarp the video or not.

		Yields
		------
		AprilTagTracker
			A tracker for Apriltags that publish messages on the /state ROS topic.
		"""

		self.camera_index = 0
		self.origin = (0, 0)
		self.state = {}  # The tuples are (x [cm], y [cm], theta [deg])

		# Set some variables
		self.number_of_tags = num
		self.cam_choice = cam_choice
		self.res_choice = res_choice
		self.tag_choice = tag_choice
		self.trail_length = trail_length
		self.save_data = save_data
		self.save_video = save_video
		self.crop_image = crop_image
		self.unwarped = unwarped

		if not os.path.exists('Data/'):
			os.mkdir('Data')

		if testname is None:
			date = datetime.datetime.now()
			self.data_dir = 'Data/' + date.strftime("%Y-%m-%d %Hh%M/")
		else:
			self.data_dir = 'Data/' + testname + "/"

		# Change the directory name if it already exist
		k = 1
		if os.path.exists(self.data_dir):
			self.data_dir = self.data_dir[:-1] + "_0/"

		while os.path.exists(self.data_dir):
			self.data_dir = self.data_dir.replace("_" + str(k - 1) + "/", "_" + str(k) + "/")
			k += 1

		self.filename = self.data_dir + "data.csv"
		self.video_name = self.data_dir + "video.avi"
		self.video_no_overlay_name = self.video_name.replace(".avi", "_no_overlay.avi")

		if self.save_data or self.save_video:
			os.mkdir(self.data_dir[:-1])

		for i in range(self.number_of_tags):
			self.state[str(i)] = (0, 0, 0)

		self.pt = ones((self.trail_length, 2, self.number_of_tags))
		self.ptmean = ones((self.trail_length, 2))
		self.arrow_vec = ones((2, self.number_of_tags))

		self.crop_params = CROP[self.cam_choice]

		# Setup the tracker
		self._tracker_setup()

		self.pub = rospy.Publisher('state', String, queue_size=2)
		rospy.init_node('AprilTags', anonymous=False)

		self.last = time()

		self.mtx = np.array([[1.12053770e+03, 0.00000000e+00, 9.63250311e+02],
						[0.00000000e+00, 1.11643457e+03, 5.33729898e+02],
						[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

		self.dist = np.array([[0.1830225, -0.40615096, -0.00123925, -0.00043419, 0.21128946]])

		self.rotate_frame_ = False

	def __del__(self):
		""" Destructor of the AprilTagTracker class. """

		self.camera.release()  # Cleanup the camera, stop saving video and close any open windows
		if self.save_video:
			self.out.release()
			self.out_no_overlay.release()
		cv2.destroyAllWindows()

	def _tracker_setup(self):
		""" Method to setup the tracker setting. """

		# Grab the reference to the camera
		self.camera = cv2.VideoCapture(self.camera_index)  # IMPORTANT: 0 for default webcam, 1 for usb webcam
		self.camera.set(3, RESOLUTION[self.res_choice][0])
		self.camera.set(4, RESOLUTION[self.res_choice][1])
		print("video received")

		w, h = RESOLUTION[self.res_choice]

		# Generate new camera matrix from parameters
		self.newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K[self.cam_choice], D[self.cam_choice], (w, h), 0)

		# Generate look-up tables for remapping the camera image
		self.mapx, self.mapy = cv2.initUndistortRectifyMap(K[self.cam_choice], D[self.cam_choice], None, self.newcameramatrix, (w, h), 5)

		screen = screeninfo.get_monitors()[0]
		cv2.namedWindow('Camera', cv2.WND_PROP_FULLSCREEN)
		cv2.moveWindow('Camera', screen.x - 1, screen.y - 1)
		cv2.setWindowProperty('Camera', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

		self.detector = at.Detector(searchpath=['apriltags3py/apriltags'], families="tag36h11", nthreads=2,
		                            quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

		if self.crop_image:
			self.dx = self.crop_params[1]
			self.dy = self.crop_params[0]
		else:
			self.dx = 0
			self.dy = 0

		if self.save_data:
			self.writer = csv.writer(open(self.filename, 'w'), quotechar='|', quoting=csv.QUOTE_MINIMAL)
			header_row = [''] * self.number_of_tags * 3
			for i in range(self.number_of_tags):
				key = str(i)
				header_row[3 * i] = 'X' + key
				header_row[3 * i + 1] = 'Y' + key
				header_row[3 * i + 2] = 'Theta' + key
			self.writer.writerow(header_row)

		if self.save_video:
			fourcc = cv2.VideoWriter_fourcc(*'XVID')
			self.out = cv2.VideoWriter(self.video_name, fourcc, 5.0, RESOLUTION[self.res_choice])
			self.out_no_overlay = cv2.VideoWriter(self.video_no_overlay_name, fourcc, 5.0, RESOLUTION[self.res_choice])

	@staticmethod
	def _is_rotation_matrix(r):
		"""Method to check if the matrix is a rotation matrix.

		Returns
		-------
		bool
			Return True if this is a rotation matrix.
		"""

		rt = transpose(r)
		should_be_identity = dot(rt, r)
		i = identity(3, dtype=r.dtype)
		n = norm(i - should_be_identity)
		return n < 1e-6

	def _rotation_matrix_to_euler_angles(self, r):
		"""Method to get euler angle from a rotation matrix.

		Returns
		-------
		ndarray
			Return the x, y and z axis rotation.
		"""

		assert (self._is_rotation_matrix(r))

		sy = sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0])

		singular = sy < 1e-6

		if not singular:
			x = atan2(r[2, 1], r[2, 2])
			y = atan2(-r[2, 0], sy)
			z = atan2(r[1, 0], r[0, 0])
		else:
			x = atan2(-r[1, 2], r[1, 1])
			y = atan2(-r[2, 0], sy)
			z = 0

		return array([x, y, z])

	def cameraRead(self):
		(grabbed, frame) = self.camera.read()
		if grabbed:
			frame = cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)

			if self.rotate_frame_:
				imdim = frame.shape[:2][::-1]
				source = np.float32([self.corners[17], self.corners[18], self.corners[19], self.corners[20]])
				destination = np.float32([[0, 0], [imdim[0], 0], [0, imdim[1]], imdim])
				M = cv2.getPerspectiveTransform(source, destination)
				frame = cv2.warpPerspective(frame, M, imdim)

		return (grabbed, frame)

	def start_tracking(self, show_mean=False, show_tags=False, show_tags_arrow=False, pub_freq=5.0, focus_frames=0):
		"""Method to start the tracker loop.

		Parameters
		----------
		show_mean : bool
			Show the trail of the mean of all the tags.
		show_tags : bool
			Show the trail of each tags.
		show_tags_arrow : bool
			Show the direction arrow of each tags.
		pub_freq : float
			Maximun publish frequency on the /state ROS topic.
		focus_frames : int
			Number of frames to ignore to let the camera focus.
		"""

		running = True
		frame_index = 0

		# Let the camera focus itself
		id_list = {}
		focus_frames = 3
		for i in range(focus_frames):
			(grabbed, frame) = self.cameraRead()

			if grabbed:
				fr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				detections = self.detector.detect(fr, estimate_tag_pose=True,
												  camera_params=CAMERA_PARAMS[self.cam_choice],
												  tag_size=TAG_SIZE[self.tag_choice])

				detections = [d for d in detections if (d.tag_id <= 20 and d.tag_id >= 17)]
				for d in detections:
					id_list[d.tag_id] = [int(d.center[0]), int(d.center[1])]

			cv2.imshow('Camera', frame)
			k = cv2.waitKey(1)
			if k == 27:
				running = False
				break
		print(id_list)
		if len(list(id_list.keys())) == 4:
			self.rotate_frame_ = True
			self.corners = id_list
		print(id_list)

		while running:
			# grab the current frame
			(grabbed, frame) = self.cameraRead()
			if not grabbed:
				break

			if self.save_video:
				self.out_no_overlay.write(frame)

			# convert frame to GRAYSCALE
			img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			if self.crop_image:  # If you crop the image, then:
				img = img[self.crop_params[0]:self.crop_params[2], self.crop_params[1]:self.crop_params[3]]
				cv2.rectangle(frame, (self.crop_params[1], self.crop_params[0]), (self.crop_params[3], self.crop_params[2]),
				              color=(0, 0, 255), thickness=4)

			# Detect tags
			detections = self.detector.detect(img, estimate_tag_pose=True, camera_params=CAMERA_PARAMS[self.cam_choice],
			                                  tag_size=TAG_SIZE[self.tag_choice])

			if frame_index == 0:  # First iteration of the loop
				ox = oy = 0
				for i, detection in enumerate(detections):
					if 0 <= detection.tag_id < self.number_of_tags:  # Make sure detection is within the tags we want
						self.pt[:, 0, detection.tag_id] *= detection.center[0] + self.dx
						self.pt[:, 1, detection.tag_id] *= detection.center[1] + self.dy
						ox += 100 * detection.pose_t[0][0]
						oy += 100 * detection.pose_t[1][0]

				self.origin = (ox / self.number_of_tags, oy / self.number_of_tags)

				self.ptmean[:, 0] *= sum(self.pt[0, 0, :])
				self.ptmean[:, 1] *= sum(self.pt[0, 1, :])

			else:
				for i, detection in enumerate(detections):
					if 0 <= detection.tag_id < self.number_of_tags:  # Make sure detection is within the tags we want
						key = str(detection.tag_id)
						self.state[key] = (100 * detection.pose_t[0][0] - self.origin[0],
						                   100 * detection.pose_t[1][0] - self.origin[1],
						                   180 * self._rotation_matrix_to_euler_angles(detection.pose_R)[2] / pi)
						self.pt[self.trail_length - 1, 0, detection.tag_id] = detection.center[0] + self.dx
						self.pt[self.trail_length - 1, 1, detection.tag_id] = detection.center[1] + self.dy
						vecx = self.pt[self.trail_length - 1, 0, detection.tag_id] - self.pt[
							self.trail_length - 2, 0, detection.tag_id]
						vecy = self.pt[self.trail_length - 1, 1, detection.tag_id] - self.pt[
							self.trail_length - 2, 1, detection.tag_id]
						veclen = npsqrt(vecx ** 2 + vecy ** 2)
						if -1 < vecx < 1 and -1 < vecy < 1:
							vecx = vecy = 0
							veclen = 1
						self.arrow_vec[0, detection.tag_id] = 20.0 * float(vecx) / veclen
						self.arrow_vec[1, detection.tag_id] = 20.0 * float(vecy) / veclen

				self.ptmean[self.trail_length - 1, 0] = sum(self.pt[self.trail_length - 1, 0, :]) / float(
					self.number_of_tags)
				self.ptmean[self.trail_length - 1, 1] = sum(self.pt[self.trail_length - 1, 1, :]) / float(
					self.number_of_tags)

			for i in range(1, self.trail_length):
				thickness = int(float(i) * 2.0 / float(self.trail_length - 1)) + 2
				if show_tags:
					for j in range(self.number_of_tags):
						cv2.line(frame, (int(self.pt[i - 1, 0, j]), int(self.pt[i - 1, 1, j])),
						         (int(self.pt[i, 0, j]), int(self.pt[i, 1, j])), (0, 255, 0), thickness)

				if show_mean:
					cv2.line(frame, (int(self.ptmean[i - 1, 0]), int(self.ptmean[i - 1, 1])),
					         (int(self.ptmean[i, 0]), int(self.ptmean[i, 1])), (255, 0, 0), thickness)

			if show_tags_arrow:
				for i in range(self.number_of_tags):
					cv2.line(frame,
					         (int(self.pt[self.trail_length - 1, 0, i]), int(self.pt[self.trail_length - 1, 1, i])),
					         (int(self.pt[self.trail_length - 1, 0, i] + self.arrow_vec[0, i]),
					          int(self.pt[self.trail_length - 1, 1, i] + self.arrow_vec[1, i])), (0, 0, 255), 2)

			freq = 1.0 / (time() - self.last)  # Cap the publish frequency to let the Photons Controller read the data
			if freq <= pub_freq:
				self.last = time()
				self.pub.publish(String(data=str(self.state)))
				if self.save_data:
					data = [0] * self.number_of_tags * 3
					for i in range(self.number_of_tags):
						key = str(i)
						data[3 * i] = self.state[key][0]
						data[3 * i + 1] = self.state[key][1]
						data[3 * i + 2] = self.state[key][2]
					self.writer.writerow(data)

			newimg = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

			if self.unwarped:
				cv2.imshow('Camera', newimg)
				if self.save_video:
					self.out.write(newimg)
			else:
				cv2.imshow('Camera', frame)
				if self.save_video:
					self.out.write(frame)

			k = cv2.waitKey(1)
			frame_index += 1

			self.ptmean = roll(self.ptmean, -1, axis=0)
			self.ptmean[-1] = self.ptmean[-2]
			for i in range(self.number_of_tags):
				self.pt[:, :, i] = roll(self.pt[:, :, i], -1, axis=0)
				self.pt[-1, :, i] = self.pt[-2, :, i]

			if k == 27:
				running = False
###################################################


###################################################
class TargetTracker:
	"""Class that use Apriltags to track the target tag positions.

	Parameters
	----------
	target_id : int
		The target Apriltag ID.
	formation : str
		The desired formation.
	dist : float
		The distance from the tag (radius) if the form is 'Around' and distance between tags for other forms.

	Attributes
	----------
	tid : int
		Target Apriltags ID.
	form : str
		The formation the robots should produce.
	dist : float
		Distance from the tag (radius) if the form is 'Around' and distance between tags for other forms.
	diff
		The position difference from the target position.
	last
		The time of the last time a message was publish on the /target ROS topic.
	pub
		The publisher used to publish on the /target ROS topic.
	"""

	def __init__(self, target_id=0, formation="None", dist=5.):
		"""Class that use data from the /state ROS topic to track the target tag positions.

		Parameters
		----------
		target_id : int
			The target Apriltag ID.
		formation : str
			The desired formation.
		dist : float
			The distance from the tag (radius) if the form is 'Around' and distance between tags for other forms.

		Yields
		------
		TargetTracker
			A class that read data from the /state ROS topic and publish messages on the /target ROS topic.
		"""

		self.tx = self.ty = 0
		self.data = {}

		# Get the id of the target tag
		self.tid = target_id
		self.form = formation
		self.dist = dist
		self.diff = self._get_form_diff()
		self.last = time()
		print()

		self.pub = rospy.Publisher('target', String, queue_size=2)
		rospy.init_node('TargetTracker', anonymous=False)
		rospy.Subscriber("state", String, self.ros_callback, queue_size=2)
		rospy.spin()

	def __del__(self):
		""" Destructor of the TargetTracker class. """

		print("\b\b  \n")

	def _get_form_diff(self):
		"""Method to get the difference from the target position of each Sphero for the Around formation.

		Returns
		-------
		ndarray
			A numpy array containing the difference from the target position of each Sphero for the Around formation.
		"""

		dang = 2. * pi / self.tid
		pos = zeros((self.tid, 2))
		for i in range(self.tid):
			ang = i * dang
			pos[i, 0] = self.dist * cos(ang)
			pos[i, 1] = self.dist * sin(ang)

		return pos

	def ros_callback(self, data):
		"""Method used get the position target for each tags. Used with a ROS Subscriber.

		Parameters
		----------
		data : String
			Message published on the /state topic containing the position of all tags.
		"""

		data_dict = eval(data.data)
		tx = data_dict[str(self.tid)][0]
		ty = data_dict[str(self.tid)][1]

		freq = 1. / (time() - self.last)
		self.last = time()

		print("\rFrequency : {0:2.2f} Hz   ".format(freq), end='')

		if tx != self.tx or ty != self.ty:  # Only update the target value if the target moved
			self.data = {}
			self.tx = tx
			self.ty = ty

			if self.form == "None":  # No formation
				# for i in range(self.tid):
				self.data[str(self.tid)] = (tx, ty)

			elif self.form == "Around":  # Formation around the target
				for i in range(self.tid):
					dx = self.diff[i, 0]
					dy = self.diff[i, 1]
					self.data[str(i)] = (tx + dx, ty + dy)

			elif self.form == "LineXP":  # Formation in line on the positive X axiskillall -9 roscore
				for i in range(self.tid):
					dx = self.dist * (i + 1)
					self.data[str(i)] = (tx + dx, ty)

			elif self.form == "LineXN":  # Formation in line on the negative X axis
				for i in range(self.tid):
					dx = self.dist * (i + 1)
					self.data[str(i)] = (tx - dx, ty)

			elif self.form == "LineYP":  # Formation in line on the positive Y axis
				for i in range(self.tid):
					dy = self.dist * (i + 1)
					self.data[str(i)] = (tx, ty + dy)

			elif self.form == "LineYN":  # Formation in line on the negative Y axis
				for i in range(self.tid):
					dy = self.dist * (i + 1)
					self.data[str(i)] = (tx, ty - dy)

			self.pub.publish(String(data=str(self.data)))
###################################################
