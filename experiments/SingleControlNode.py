""" Python script used to control a single Sphero Mini/Bolt with ROS. """

####################################################
import argparse
import signal
from Jameoba import SpheroMini, SpheroBolt, SpheroController
####################################################


def signal_handler(sig, frame):
	exit()


# Create the out of the loop interrupt
signal.signal(signal.SIGINT, signal_handler)

# Parse the arguments
parser = argparse.ArgumentParser(description='Start a control node for a single Jameoba Sphero robot')
parser.add_argument('id', type=int, help='Apriltag id of the Sphero to control')
parser.add_argument('-i', '--init', action="store_true", default=False,
					help='Put this tag to initialise the direction of the robot')
parser.add_argument('-s', '--simul', action="store_true", default=False,
					help='Put this tag to make a simulation without a Sphero')
parser.add_argument('-b', '--bolt', action="store_true", default=False,
					help='Put this tag if you use a Sphero Bolt')
parser.add_argument('-c', '--color', type=int, nargs="+", default=[0, 0, 255],
					help='Change the color. Default is blue.')
parser.add_argument('-g', '--gain', type=float, nargs="+", default=[1.5, 0.1, 0.],
					help='Change the gains in this order : kp, ki, kd. Default is kp=1.5, ki=0.1 and kd=0.')
args = parser.parse_args()


# Is it just a simulation or the real deal?
if bool(args.simul):
	sphero = args.id
else:
	if bool(args.bolt):
		sphero = SpheroBolt(tag_id=args.id, verbosity=0, try_num=3)
	else:
		sphero = SpheroMini(tag_id=args.id, verbosity=0, try_num=3)

	# Set the desired color
	if len(tuple(args.color)) == 3:
		sphero.set_color(rgb=args.color)

# Initialise the zeros heading of the Sphero
if bool(args.init) and not bool(args.simul):
	sphero.init_heading()

# Set the controller gains
if len(args.gain) == 3:
	kp = args.gain[0]
	ki = args.gain[1]
	kd = args.gain[2]
else:
	kp = 1.5
	ki = 0.1
	kd = 0.

# Initialize the controller
ctl = SpheroController(sphero, kp=kp, ki=ki, kd=kd)
