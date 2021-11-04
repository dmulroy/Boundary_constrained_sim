""" Python script used to control multiple Sphero Bolt with ROS. """

####################################################
import signal
import argparse
from Jameoba import MultiSpheroBolt, SpheroController
####################################################


def signal_handler(sig, frame):
	exit()


# Create the out of the loop interrupt
signal.signal(signal.SIGINT, signal_handler)

# Parse the arguments
parser = argparse.ArgumentParser(description='Start a control node for multiple Jameoba Sphero robots')
parser.add_argument('ids', type=int, nargs="+", help='Apriltag id of the Spheros to control')
parser.add_argument('-i', '--init', action="store_true", default=False,
					help='Put this tag to initialise the direction of the robots')
parser.add_argument('-c', '--color', type=int, nargs="+", default=[0, 0, 255],
					help='Change the color. Default is blue.')
parser.add_argument('-k', '--channel', type=int, default=0,
					help='The channel on which to broadcast')
parser.add_argument('-g', '--gain', type=float, nargs="+", default=[1., 0.1, 0.],
					help='Change the gains in this order : kp, ki, kd. Default is kp=1., ki=0.1 and kd=0.')
args = parser.parse_args()

multi_sphero = MultiSpheroBolt(tag_ids=tuple(args.ids), verbosity=0, try_num=3)

# Initialise the zeros heading of the Spheros
if bool(args.init):
	multi_sphero.init_heading()

# Set the desired color
if len(tuple(args.color)) == 3:
	multi_sphero.set_colors(color=args.color)

multi_sphero.setup_leader_followers(channel=args.channel)

# Set the controller gains
if len(args.gain) == 3:
	kp = args.gain[0]
	ki = args.gain[1]
	kd = args.gain[2]
else:
	kp = 1.
	ki = 0.1
	kd = 0.

# Initialize the controller
ctl = SpheroController(multi_sphero.spheros[0], kp=kp, ki=ki, kd=kd)
