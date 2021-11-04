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

	def __init__(self, sphero1, sphero2, title="Vibration manual control"):
		""" Initialisation of the class """
		self.sphero1 = sphero1
		self.sphero2 = sphero2
		self.heading = 0
		self.speed = 0
		self.sleep = False
		self.auto_send = IntVar()
		self.window = Tk()
		self.window.title(title)
		self._create_widget()
		self.window.mainloop()

	def _create_widget(self):
		""" Method that create the widget of the app """

		self.lab1 = Label(self.window)
		self.lab1['text'] = 'Heading'
		self.lab1.grid(row=0, column=0)

		self.headScl = Scale(self.window, from_=0, to=360)
		self.headScl["command"] = self.set_heading
		self.headScl.grid(row=1, column=0, rowspan=3)

		self.lab2 = Label(self.window)
		self.lab2['text'] = 'Speed'
		self.lab2.grid(row=0, column=1)

		self.speedScl = Scale(self.window, from_=0, to=255)
		self.speedScl["command"] = self.set_speed
		self.speedScl.grid(row=1, column=1, rowspan=3)

		self.sendbut = Button(self.window)
		self.sendbut['text'] = 'SEND'
		self.sendbut['command'] = self.send
		self.sendbut.grid(row=4, column=0, columnspan=2)

		self.stopbut = Button(self.window)
		self.stopbut["text"] = "STOP"
		self.stopbut["command"] = self.stop
		self.stopbut.grid(row=5, column=0, columnspan=2)

		self.resetbut = Button(self.window)
		self.resetbut['text'] = 'RESET HEADING'
		self.resetbut['command'] = self.reset
		self.resetbut.grid(row=6, column=0, columnspan=2)

		self.sleepbut = Button(self.window)
		self.sleepbut["text"] = "SLEEP"
		self.sleepbut["command"] = self.sleep
		self.sleepbut.grid(row=7, column=0, columnspan=2)

		self.autobut = Checkbutton(self.window)
		self.autobut['text'] = "AUTOSEND"
		self.autobut["variable"] = self.auto_send
		self.autobut.grid(row=8, column=0, columnspan=2)

	def set_heading(self):
		""" Set the heading without sending it """
		self.heading = self.headScl.get()
		if self.auto_send:
			self.send()

	def set_speed(self):
		""" Set the speed without sending it """
		self.speed = self.speedScl.get()
		if self.auto_send:
			self.send()

	def send(self):
		""" Send the heading and speed """
		self.sphero1.roll(speed=self.speed, heading=self.heading)
		self.sphero2.roll(speed=self.speed, heading=self.heading)

	def stop(self):
		""" Stop the spheros """
		self.sphero1.roll(speed=0, heading=self.heading)
		self.sphero2.roll(speed=0, heading=self.heading)

	def reset(self):
		""" Reset the heading of the robots """
		self.headScl.set(0)
		self.sphero1.resetHeading()
		self.sphero2.resetHeading()

	def sleep(self):
		""" Set the spheros to sleep """
		if self.sleep:
			self.sphero1.wake()
			self.sphero2.wake()
			self.sleep = False
			self.sleepbut['text'] = "SLEEP"
		else:
			self.sphero1.sleep()
			self.sphero2.sleep()
			self.sleep = True
			self.sleepbut['text'] = "WAKE"
###################################################


###################################################
class TargetSetApp(object):
	"""
	The application that will set the target
	coordinate of a sphero
	"""

	def __init__(self, at_id):
		""" Method to initialise the application """
		# Set variables
		self.ID = str(at_id)
		self.data = {"data": (0, 0)}  # (x, y)

		# Set the ROS node
		node_name = "Target_ID" + str(at_id)
		rospy.init_node(node_name, anonymous=False)
		topic = "spheros_target/" + str(at_id)
		self.pub = rospy.Publisher(topic, String, queue_size=10)

		# Set the tkinter GUI
		self.window = Tk()
		self.canvas = Canvas(self.window, height=500, width=500)
		self.canvas.pack()
		self.canvas.bind('<1>', self.movement)  # the magic
		self.canvas.bind('<B1-Motion>', self.movement)  # similar magic
		self.canvas.bind('<ButtonRelease-1>', self.save)  # result of magic
		self._draw_bg()
		self.indicator = self.canvas.create_oval(245, 245, 255, 255, fill='red')  # dot
		self.coord = self.canvas.create_text(260, 250, anchor=W, text="(0, 0)")
		self.window.mainloop()

	def _draw_bg(self):
		self.canvas.create_rectangle(30, 30, 470, 470, fill='white', outline='white')
		self.canvas.create_line(30, 30, 30, 480)
		self.canvas.create_line(20, 30, 40, 30)
		self.canvas.create_line(20, 250, 40, 250)
		self.canvas.create_line(20, 470, 470, 470)
		self.canvas.create_line(250, 460, 250, 480)
		self.canvas.create_line(470, 460, 470, 480)
		self.canvas.create_text(250, 15, anchor=CENTER, text="Controlling Sphero with ID: " + self.ID)

	def movement(self, event):  # move the dot
		self.canvas.coords(self.indicator, event.x-5, event.y-5, event.x+5, event.y+5)
		x = (event.x - 250)/8
		y = (event.y - 250)/8
		val = "(" + str(x) + ", " + str(y) + ")"
		self.canvas.coords(self.coord, event.x+10, event.y)
		self.canvas.itemconfig(self.coord, text=val)

	def save(self, event):  # report the dot
		self.data["data"] = ((event.x - 250)/8, (event.y - 250)/8)
		self.pub.publish(str(self.data))
###################################################


###################################################
class HeadingSetApp(object):
	"""
	The application that will control a sphero
	"""

	def __init__(self, address_list=None):
		""" Method to initialise the application """
		if address_list is None:
			address_list = [""]

		self.number_of_robots = len(address_list)  # Number of robots

		self.sphero_list = []  # Initialize the Sphero list
		for MAC in address_list:
			sphero = sphero_mini(MAC, verbosity=1)
			self.sphero_list.append(sphero)

		# Set variables
		self.window = Tk()
		self.ID_var = IntVar(self.window)
		self.ID_var.set(0)
		self.ID_var.trace("w", self.change_var)
		self.sphero_index = self.ID_var.get()
		self.speed = 0
		self.heading = 0
		self.last = time()

		# Set the tkinter GUI
		self._build_widgets()

	def __del__(self):
		""" Deletion of the instance """
		for sphero in self.sphero_list:
			sphero.sleep()
			sphero.disconnect()

	def _build_widgets(self):
		""" This method build the widget and the app """
		self.canvas = Canvas(self.window, height=500, width=500)
		self.canvas.grid(row=0, column=0, columnspan=(self.number_of_robots + 1))
		self.canvas.bind('<1>', self.movement)  # the magic
		self.canvas.bind('<B1-Motion>', self.movement)  # similar magic
		self.canvas.bind('<ButtonRelease-1>', self.save)  # result of magic
		self.canvas.create_oval(30, 30, 470, 470, fill='white', outline='black')
		self.canvas.create_text(250, 15, anchor=CENTER, text="Sphero Joystick control")
		self.indicator = self.canvas.create_oval(215, 215, 285, 285, fill='red')  # dot

		self.reset_heading = Button(self.window)
		self.reset_heading['text'] = "RESET HEADING"
		self.reset_heading['command'] = self.reset_h
		self.reset_heading.grid(row=1, column=0)

		self.radio_but_array = {"0": Radiobutton(self.window, text="ID: 0", variable=self.ID_var, value=0)}
		self.radio_but_array["0"].grid(row=1, column=1)
		for i in range(1, self.number_of_robots):
			self.radio_but_array[str(i)] = Radiobutton(self.window, text=("ID: " + str(i)), variable=self.ID_var, value=i)
			self.radio_but_array[str(i)].grid(row=1, column=i+1)

	def movement(self, event):  # move the dot
		x = event.x - 250
		y = event.y - 250
		self.canvas.coords(self.indicator, event.x - 35, event.y - 35, event.x + 35, event.y + 35)

		self.speed = int(sqrt(x ** 2 + y ** 2))
		if self.speed >= 255:
			self.speed = 255
		self.heading = int(180 * atan2(y, x) / pi) + 180

		self.send_data()

	def save(self, event):  # report the dot
		self.canvas.coords(self.indicator, 215, 215, 285, 285)

		self.speed = 0

		self.send_data()

	def send_data(self):
		freq = 1.0 / (time() - self.last)
		if freq <= 5.0:
			self.sphero_list[self.sphero_index].roll(self.speed, self.heading)

	def reset_h(self):
		self.sphero_list[self.sphero_index].resetHeading()

	def change_var(self, arg1, arg2, arg3):
		self.sphero_index = self.ID_var.get()

	def start(self):
		self.window.mainloop()
###################################################


###################################################
class TwoTargetSetApp(object):
	"""
	The application that will set the target
	coordinate of a sphero
	"""

	def __init__(self):
		""" Method to initialise the application """
		# Set variables
		self.data = {"0": (0, 0), "1": (0, 0)}  # (x, y)

		# Set the ROS node
		rospy.init_node("target_set", anonymous=False)
		self.pub = rospy.Publisher("sphero_target", String, queue_size=10)

		# Set the tkinter GUI
		self.window = Tk()
		self.canvas1 = Canvas(self.window, height=500, width=500)
		self.canvas1.grid(row=0, column=0)
		self.canvas2 = Canvas(self.window, height=500, width=500)
		self.canvas2.grid(row=0, column=1)
		self._bind_canvas()
		self._draw_bg()
		self.indicator1 = self.canvas1.create_oval(245, 245, 255, 255, fill='red')  # dot
		self.indicator2 = self.canvas2.create_oval(245, 245, 255, 255, fill='red')  # dot
		self.window.mainloop()

	def _bind_canvas(self):
		self.canvas1.bind('<1>', self.movement1)  # the magic
		self.canvas1.bind('<B1-Motion>', self.movement1)  # similar magic
		self.canvas1.bind('<ButtonRelease-1>', self.save1)  # result of magic
		self.canvas2.bind('<1>', self.movement2)  # the magic
		self.canvas2.bind('<B1-Motion>', self.movement2)  # similar magic
		self.canvas2.bind('<ButtonRelease-1>', self.save2)  # result of magic

	def _draw_bg(self):
		# Set first
		self.canvas1.create_rectangle(30, 30, 470, 470, fill='white', outline='white')
		self.canvas1.create_line(30, 30, 30, 480)
		self.canvas1.create_line(20, 30, 40, 30)
		self.canvas1.create_line(20, 250, 40, 250)
		self.canvas1.create_line(20, 470, 470, 470)
		self.canvas1.create_line(250, 460, 250, 480)
		self.canvas1.create_line(470, 460, 470, 480)
		self.canvas1.create_text(250, 15, anchor=CENTER, text="Controlling Sphero with ID: 0")
		# Set second
		self.canvas2.create_rectangle(30, 30, 470, 470, fill='white', outline='white')
		self.canvas2.create_line(30, 30, 30, 480)
		self.canvas2.create_line(20, 30, 40, 30)
		self.canvas2.create_line(20, 250, 40, 250)
		self.canvas2.create_line(20, 470, 470, 470)
		self.canvas2.create_line(250, 460, 250, 480)
		self.canvas2.create_line(470, 460, 470, 480)
		self.canvas2.create_text(250, 15, anchor=CENTER, text="Controlling Sphero with ID: 1")

	def movement1(self, event):  # move the dot
		self.canvas1.coords(self.indicator1, event.x - 5, event.y - 5, event.x + 5, event.y + 5)

	def save1(self, event):  # report the dot
		self.data["0"] = ((event.x - 250) / 8, (event.y - 250) / 8)
		self.pub.publish(str(self.data))

	def movement2(self, event):  # move the dot
		self.canvas2.coords(self.indicator2, event.x - 5, event.y - 5, event.x + 5, event.y + 5)

	def save2(self, event):  # report the dot
		self.data["1"] = ((event.x - 250) / 8, (event.y - 250) / 8)
		self.pub.publish(str(self.data))
###################################################


###################################################
class MultiTargetSetApp(object):
	"""
	The application that will set the target
	coordinate of multiple spheros
	"""

	def __init__(self, number_of_sphero=1):
		""" Method to initialise the application """
		# Set variables
		self.num = number_of_sphero
		self.canvas_size = 900.
		self.area = 30.

		if self.num < 5:
			rspan = 5
		else:
			rspan = self.num

		# Set the data output
		self.data = {"0": (0, 0)}  # (x, y)
		for i in range(self.num):
			self.data[str(i)] = (0, 0)

		# Set the ROS node
		rospy.init_node("target_set", anonymous=False)
		self.pub_tar = rospy.Publisher("sphero_target", String, queue_size=10)
		self.pub_gain = rospy.Publisher("sphero_gains", String, queue_size=10)
		rospy.Subscriber("sphero_position", String, self.draw_current_position)

		# Set the tkinter GUI
		self.window = Tk()
		self.ID_var = IntVar(self.window)
		self.ID_var.set(0)
		self.ID_var.trace("w", self.change_var)
		self.canvas = Canvas(self.window, height=self.canvas_size, width=self.canvas_size)
		self.canvas.grid(row=0, column=1, rowspan=rspan)
		self.radio_but_array = {"0": Radiobutton(self.window, text="ID: 0", variable=self.ID_var, value=0)}
		self.radio_but_array["0"].grid(row=0, column=0)
		for i in range(1, self.num):
			self.radio_but_array[str(i)] = Radiobutton(self.window, text=("ID: " + str(i)), variable=self.ID_var, value=i)
			self.radio_but_array[str(i)].grid(row=i, column=0)
		self._bind_canvas()
		self._draw_bg()
		self._set_scale()
		self.indicator = self.canvas.create_oval(self.canvas_size/2 - 5, self.canvas_size/2 - 5,
												 self.canvas_size/2 + 5, self.canvas_size/2 + 5, fill='red')  # dot
		self.indicator_current = self.canvas.create_oval(self.canvas_size/2 - 5, self.canvas_size/2 - 5,
														 self.canvas_size/2 + 5, self.canvas_size/2 + 5, fill='gray')  # dot
		self.window.mainloop()

	def _bind_canvas(self):
		self.canvas.bind('<1>', self.movement)  # the magic
		self.canvas.bind('<B1-Motion>', self.movement)  # similar magic
		self.canvas.bind('<ButtonRelease-1>', self.save)  # result of magic

	def _draw_bg(self):
		# Set first
		self.canvas.create_rectangle(30, 30, self.canvas_size - 30, self.canvas_size - 30, fill='white', outline='white')
		self.canvas.create_line(30, 30, 30, self.canvas_size - 20)
		self.canvas.create_line(20, 30, 40, 30)
		self.canvas.create_line(20, self.canvas_size/2, 40, self.canvas_size/2)
		self.canvas.create_line(20, self.canvas_size - 30, self.canvas_size - 30, self.canvas_size - 30)
		self.canvas.create_line(self.canvas_size/2, self.canvas_size - 40, self.canvas_size/2, self.canvas_size - 20)
		self.canvas.create_line(self.canvas_size - 30, self.canvas_size - 40, self.canvas_size - 30, self.canvas_size - 20)
		self.canvas.create_text(self.canvas_size/2, 15, anchor=CENTER, text="Controlling multiple sphero(s)")

	def _set_scale(self):
		# Set the variables
		self.kp = DoubleVar(self.window)
		self.kp.set(0.0)
		self.ki = DoubleVar(self.window)
		self.ki.set(0.0)
		self.kd = DoubleVar(self.window)
		self.kd.set(0.0)
		self.ff = DoubleVar(self.window)
		self.ff.set(0.0)

		self.kplb = Label(self.window)
		self.kplb['text'] = 'P gain'
		self.kplb.grid(row=0, column=2)

		self.kp_scale = Scale(self.window, from_=0, to=5, resolution=0.1)
		self.kp_scale["variable"] = self.kp
		self.kp_scale["orient"] = HORIZONTAL
		self.kp_scale.grid(row=0, column=3)

		self.kilb = Label(self.window)
		self.kilb['text'] = 'I gain'
		self.kilb.grid(row=1, column=2)

		self.ki_scale = Scale(self.window, from_=0, to=0.15, resolution=0.001)
		self.ki_scale["variable"] = self.ki
		self.ki_scale["orient"] = HORIZONTAL
		self.ki_scale.grid(row=1, column=3)

		self.kdlb = Label(self.window)
		self.kdlb['text'] = 'D gain'
		self.kdlb.grid(row=2, column=2)

		self.kd_scale = Scale(self.window, from_=0, to=0.15, resolution=0.001)
		self.kd_scale["variable"] = self.kd
		self.kd_scale["orient"] = HORIZONTAL
		self.kd_scale.grid(row=2, column=3)

		self.fflb = Label(self.window)
		self.fflb['text'] = 'Feed forward'
		self.fflb.grid(row=3, column=2)

		self.ff_scale = Scale(self.window, from_=0, to=20)
		self.ff_scale["variable"] = self.ff
		self.ff_scale["orient"] = HORIZONTAL
		self.ff_scale.grid(row=3, column=3)

		self.send_but = Button(self.window)
		self.send_but["text"] = "SEND"
		self.send_but["command"] = self.send_gain
		self.send_but.grid(row=4, column=2)

		self.stop_but = Button(self.window)
		self.stop_but["text"] = "STOP"
		self.stop_but["command"] = self.stop_gain
		self.stop_but.grid(row=4, column=3)

	def movement(self, event):  # move the dot
		self.canvas.coords(self.indicator, event.x - 5, event.y - 5, event.x + 5, event.y + 5)

	def save(self, event):  # report the dot
		self.data[str(self.ID_var.get())] = ((2*event.x/self.canvas_size - 1)*self.area,
											 (2*event.y/self.canvas_size - 1)*self.area)
		self.pub_tar.publish(str(self.data))

	def draw_current_position(self, data):
		data_dict = eval(data.data)
		x = int((data_dict[str(self.ID_var.get())][0]/self.area + 1.)*self.canvas_size/2.)
		y = int((data_dict[str(self.ID_var.get())][1]/self.area + 1.)*self.canvas_size/2.)
		self.canvas.coords(self.indicator_current, x - 5, y - 5, x + 5, y + 5)

	def change_var(self, arg1, arg2, arg3):
		x = int((self.data[str(self.ID_var.get())][0]/self.area + 1.)*self.canvas_size/2.)
		y = int((self.data[str(self.ID_var.get())][1]/self.area + 1.)*self.canvas_size/2.)
		self.canvas.coords(self.indicator, x - 5, y - 5, x + 5, y + 5)

	def send_gain(self):
		gain = {"kp": self.kp.get(),
				"ki": self.ki.get(),
				"kd": self.kd.get(),
				"ff": self.ff.get()}
		self.pub_gain.publish(str(gain))

	def stop_gain(self):
		gain = {"kp": 0,
				"ki": 0,
				"kd": 0,
				"ff": 0}
		self.pub_gain.publish(str(gain))
###################################################
