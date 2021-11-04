"""
Python class used to set
up applications for the project
"""

###################################################
try:
	from tkinter import Tk, IntVar, DoubleVar, Label, \
		Scale, Button, Checkbutton, Radiobutton, Canvas, \
		W, CENTER, HORIZONTAL
except ImportError:
	from Tkinter import Tk, IntVar, DoubleVar, Label, \
		Scale, Button, Checkbutton, Radiobutton, Canvas, \
		W, CENTER, HORIZONTAL
from math import sqrt, atan2, pi
import rospy
from std_msgs.msg import String
from Sphero_mini.sphero_mini import sphero_mini
from time import time
###################################################


###################################################
class SpheroApplication:
	"""
	The application that will control two spheros
	"""

	def __init__(self, tag_ids, title="Vibration manual control"):
		""" Initialisation of the class """

		self.tag_ids = tag_ids
		self.state = {}
		for tag in self.tag_ids:
			self.state[str(tag)] = (0, 0, 0)

		self.window = Tk()
		self.speed = IntVar(self.window)
		self.speed.set(0)
		self.window.title(title)
		self._create_widget()
		self.pub1 = rospy.Publisher("state", String, queue_size=2)
		self.pub2 = rospy.Publisher("target", String, queue_size=2)
		rospy.init_node("ManCTL", anonymous=False)
		self.window.mainloop()

	def _create_widget(self):
		""" Method that create the widget of the app """

		self.lab1 = Label(self.window)
		self.lab1['text'] = 'Speed'
		self.lab1.grid(row=5, column=0, columnspan=3)

		self.headScl = Scale(self.window, from_=0, to=10)
		self.headScl["variable"] = self.speed
		self.headScl["orient"] = HORIZONTAL
		self.headScl.grid(row=4, column=0, columnspan=3)

		self.increasex = Button(self.window)
		self.increasex['text'] = '+X'
		self.increasex.bind("<ButtonPress>", self.plusx)
		self.increasex.bind("<ButtonRelease>", self.release)
		self.increasex.grid(row=1, column=0)

		self.decreasex = Button(self.window)
		self.decreasex["text"] = "-X"
		self.decreasex.bind("<ButtonPress>", self.minusx)
		self.decreasex.bind("<ButtonRelease>", self.release)
		self.decreasex.grid(row=1, column=2)

		self.increasey = Button(self.window)
		self.increasey['text'] = '+Y'
		self.increasey.bind("<ButtonPress>", self.plusy)
		self.increasey.bind("<ButtonRelease>", self.release)
		self.increasey.grid(row=0, column=1)

		self.decreasey = Button(self.window)
		self.decreasey["text"] = "-Y"
		self.decreasey.bind("<ButtonPress>", self.minusy)
		self.decreasey.bind("<ButtonRelease>", self.release)
		self.decreasey.grid(row=2, column=1)

	def release(self, args):
		""" increases x-position"""

		msg = String(data=str(self.state))

		self.pub2.publish(msg)
		self.pub1.publish(msg)

	def plusx(self, args):
		""" increases x-position"""
		data = {}
		for tag in self.tag_ids:
			data[str(tag)] = (self.headScl.get(), 0, 0)

		msg1 = String(data=str(data))

		self.pub2.publish(msg1)

		msg2 = String(data=str(self.state))

		self.pub1.publish(msg2)

	def minusx(self, args):
		""" decreases x-position """
		data = {}
		for tag in self.tag_ids:
			data[str(tag)] = (-self.headScl.get(), 0, 0)

		msg1 = String(data=str(data))

		self.pub2.publish(msg1)

		msg2 = String(data=str(self.state))

		self.pub1.publish(msg2)

	def plusy(self, args):
		""" increases y-position """
		data = {}
		for tag in self.tag_ids:
			data[str(tag)] = (0, self.headScl.get(), 0)

		msg1 = String(data=str(data))

		self.pub2.publish(msg1)

		msg2 = String(data=str(self.state))

		self.pub1.publish(msg2)

	def minusy(self, args):
		""" decreases y-position """
		data = {}
		for tag in self.tag_ids:
			data[str(tag)] = (0, -self.headScl.get(), 0)

		msg1 = String(data=str(data))

		self.pub2.publish(msg1)

		msg2 = String(data=str(self.state))

		self.pub1.publish(msg2)

###################################################


if __name__ == "__main__":
	app = SpheroApplication(tag_ids=(9, 10, 11))
