"""
This module contains tools to control
Sphero(s) Mini/Bolt and track them using
Apriltags. This is to be used with ROS.
"""

from .Spheros import MINI, BOLT, MAC_ADDRESS, SpheroMini, SpheroBolt, MultiSpheroMini, MultiSpheroBolt
from .Controllers import SpheroController, MultiSpheroController

try:
	from .Tracker import TargetTracker, AprilTagTracker
except ImportError:
	print("Can't import AprilTagTracker and TargetTracker... "
		  "If this is not the computer you'll use for the AprilTags tracking, you can ignore this.")

try:
	from .Applications import SpheroApplication, TargetSetApp, HeadingSetApp, TwoTargetSetApp, MultiTargetSetApp
except ImportError:
	print("Can't import Applications... You can ignore if using a Raspberry Pi.")

