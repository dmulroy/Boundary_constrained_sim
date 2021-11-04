""" Python script used to control multiple Sphero Mini/Bolt with ROS. """

####################################################
import signal
import argparse
from Jameoba import MultiSpheroMini, MultiSpheroBolt, MultiSpheroController
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
parser.add_argument('-s', '--simul', action="store_true", default=False,
					help='Put this tag to make a simulation without a Sphero')
parser.add_argument('-b', '--bolt', action="store_true", default=False,
					help='Put this tag if you use a Sphero Bolt')
parser.add_argument('-c', '--color', type=int, nargs="+", default=[0, 0, 255],
					help='Change the color. Default is blue.')
parser.add_argument('-g', '--gain', type=float, nargs="+", default=[1., 0.1, 0.],
					help='Change the gains in this order : kp, ki, kd. Default is kp=1., ki=0.1 and kd=0.')
parser.add_argument('-pot', '--pot', action="store_true", default=False, help='Put this to use potential fields')
parser.add_argument('-pid', '--pid', action="store_true", default=False, help='Put this to use pid controller')
args = parser.parse_args()

# Is it just a simulation or the real deal?
if bool(args.simul):
	multi_sphero = tuple(args.ids)
else:
	if bool(args.bolt):
		multi_sphero = MultiSpheroBolt(tag_ids=tuple(args.ids), verbosity=0, try_num=3)
	else:
		multi_sphero = MultiSpheroMini(tag_ids=tuple(args.ids), verbosity=0, try_num=3)

	# Initialise the zeros heading of the Spheros
	if bool(args.init):
		multi_sphero.init_heading()

	# Set the desired color
	if len(tuple(args.color)) == 3:
		multi_sphero.set_colors(color=args.color)

# Set the controller gains
if len(args.gain) == 3:
	kp = args.gain[0]
	ki = args.gain[1]
	kd = args.gain[2]
else:
	kp = 1.
	ki = 0.1
	kd = 0.

if args.pid:
	method = 'pid'
elif args.pot:
	method = 'pot'
else:
	method = 'pid'

# Initialize the controller
ctl = MultiSpheroController(multi_sphero, kp=kp, ki=ki, kd=kd, method=method)
