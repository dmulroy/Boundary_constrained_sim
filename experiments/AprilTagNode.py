""" Python script used to track position of the tags. """

####################################################
import argparse
from Jameoba.Tracker import AprilTagTracker, CAM_BRIO, RES_1080P
####################################################

# Parse the arguments
parser = argparse.ArgumentParser(description='Tracking module for Jameoba')
parser.add_argument('num', type=int, help="Number of tags to track (If you are tracking the target,"
                                          "don't forget to include it in your count)")
parser.add_argument('-f', '--freq', type=int, default=60, help="Max refresh frequency for the tags detection.")
parser.add_argument('-u', '--unwarp', action="store_true", default=False,
                    help='Put this tag to unwarp the shown image.')
parser.add_argument('-s', '--save', action="store_true", default=False,
                    help='Put this tag to save the video and the data.')
args = parser.parse_args()

# Initialize the Apriltag tracker
tracker = AprilTagTracker(num=args.num, save_data=args.save, save_video=args.save, crop_image=False,
                          cam_choice=CAM_BRIO, res_choice=RES_1080P, unwarped=args.unwarp)

# Start the tracking

tracker.start_tracking(show_mean=False, show_tags=True, show_tags_arrow=True, focus_frames=100,pub_freq=args.freq)







