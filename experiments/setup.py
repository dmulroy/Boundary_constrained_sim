from setuptools import setup

setup(
   name='SpheroJameoba',
   version='0.1',
   description='A python project to use the Orbotix Sphero Mini in a swarm with ROS for the Jameoba project.',
   author='Bruno-Pier Busque',
   author_email='bruno-pier.busque@usherbrooke.ca',
   packages=['Jameoba'],
   install_requires=['screeninfo', 'numpy', 'opencv-python', 'pid_controller', 'bluepy', 'pysphero'],
   scripts=[
            'AprilTagNode.py',
            'SingleControlNode.py',
            'MultiControlNode.py',
            'TargetTrackNode.py',
           ]
)
