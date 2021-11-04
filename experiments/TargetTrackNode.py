""" Python script used to get the target position.

    """

####################################################
import argparse
from Jameoba.Tracker import TargetTracker
####################################################

# Parse the arguments
parser = argparse.ArgumentParser(description='Tracking module for Jameoba')
parser.add_argument('target_id', type=int, help='Apriltag id of the target')
parser.add_argument('-f', '--form', type=str, default="None",
                    help='Put this tag to have the Spheros keep a formation.')
parser.add_argument('-r', '--radius', type=int, default=5,
                    help='The radius of the circle or the distance between robots.')
args = parser.parse_args()

# Initialise the tracker
tt = TargetTracker(target_id=args.target_id, formation=args.form, dist=args.radius)
