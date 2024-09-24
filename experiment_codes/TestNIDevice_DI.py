
import nidaqmx
from time import time, sleep
from numpy import array, matmul
from numpy.fft import fft, ifft, fftshift, ifftshift
import numpy as np
import matplotlib.pyplot as plt
from nidaqmx.constants import Edge, AcquisitionType, LineGrouping
from Script_Declan import *
from copy import copy
from datetime import datetime
import pandas as pd
import nidaqmx
import numpy as np
from nidaqmx import *

class NI_Device:
    dev_name = 'NI USB-6343'
    task = ''

    N_SAMPLES = 500
    FREQ = 1e6
    N_SENSORS = 20

    def __init__(self, frequency=1e6, samples=600):

        self.FREQ = frequency
        self.N_SAMPLES = samples

        self.x_data = np.linspace(0, self.FREQ, self.N_SAMPLES)

        self.offsets = np.zeros(self.N_SENSORS)
        self.kForce = -.01

        self.last_samples = np.zeros(self.N_SENSORS)
        self.a = .2

        self.task = nidaqmx.Task()
        self.task.di_channels.add_di_chan('Dev1/port0/line0:23')
        self.task.timing.cfg_samp_clk_timing(self.FREQ, sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                                             source="", samps_per_chan=self.N_SAMPLES)

    def readSample(self, n):
        return self.task.read(number_of_samples_per_channel=1000)

    def readSingleData(self):
        return self.task.read()

    def getFFTData(self):
        data = [t.read(self.N_SAMPLES) for t in self.task]
        data = np.array(data) * 1

        fft_data = [np.abs(fft(d)) for d in data]

        return fft_data

    def getMainFreq(self):
        data = self.task.read(self.N_SAMPLES)

        data_diff = np.array(data[1:]) ^ np.array(data[:-1])

        freq = []
        for i in range(self.N_SENSORS):
            data_tmp = data_diff & (0b1 << i)

            idx1 = np.nanargmax(data_tmp)
            idx2 = np.nanargmax(data_tmp[idx1 + 1:])
            idx3 = np.nanargmax(data_tmp[idx1 + idx2 + 2:])
            period = (idx3 + idx2) / self.FREQ
            if period == 0:
                period = 1e15

            freq.append(1 / period)

        return np.array(freq)

    def calculateOffset(self):
        offsets = np.zeros(self.N_SENSORS)
        for i in range(20):
            data = self.getMainFreq()
            for j in range(self.N_SENSORS):
                offsets[j] += data[j] / 20.

        self.offsets = offsets
        self.last_samples = np.copy(self.offsets)

    def getForceMetric(self):

        data = self.last_samples * (1. - self.a) + self.a * np.array(self.getMainFreq())

        self.last_samples = copy(data)
        for i in range(self.N_SENSORS):
            data[i] -= self.offsets[i]
            data[i] *= self.kForce
            data[i] = max(0., data[i])

        return data


if __name__ == "__main__":
    NIdev = NI_Device()

    count = 1

    T0 = time()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    lines = []
    for i in range(1):
        line1, = ax.plot([], [])  # Returns a tuple of line objects, thus the comma
        lines.append(line1)

    dataHist = []
    timeHist = []
    metricHist = []

    plt.rcParams["keymap.quit"] = ["ctrl+w", "cmd+w", "q"]
    def on_key_press(event):
        if event.key == 'r':
            print("Recording data!")
            # Save the data to a CSV file with current date and time as filename
            now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"data_{now}.csv"
            df = pd.DataFrame({'x': timeHist, 'Metric': metricHist})

            for i in range(20):
                df[f'y{i + 1}'] = np.array(dataHist)[:, i]
            df.to_csv('Outputs/' + filename, index=False)
            print(f'Data saved to {filename}')
            plt.close()
        elif event.key == 'd':
            with nidaqmx.Task() as task:
                task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
                task.write(5.0)
                task.stop()

            task2 = nidaqmx.Task()
            task2.do_channels.add_do_chan('Dev1/port1/line1')
            task2.write(True)  # True is out, False is in

            test_Task = nidaqmx.Task()
            test_Task.ao_channels.add_ao_voltage_chan('Dev1/ao1')
            test_Task.timing.cfg_samp_clk_timing(rate=1500, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                                 samps_per_chan=5)
            test_Writer = nidaqmx.stream_writers.AnalogSingleChannelWriter(test_Task.out_stream, auto_start=True)
            samples = np.array([0., 5.] * 10)
            test_Writer.write_many_sample(samples)
        elif event.key == 'a':
            with nidaqmx.Task() as task:
                task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
                task.write(5.0)
                task.stop()

            task2 = nidaqmx.Task()
            task2.do_channels.add_do_chan('Dev1/port1/line1')
            task2.write(False)  # True is out, False is in

            test_Task = nidaqmx.Task()
            test_Task.ao_channels.add_ao_voltage_chan('Dev1/ao1')
            test_Task.timing.cfg_samp_clk_timing(rate=1500, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                                 samps_per_chan=5)
            test_Writer = nidaqmx.stream_writers.AnalogSingleChannelWriter(test_Task.out_stream, auto_start=True)
            samples = np.array([0., 5.] * 10)
            test_Writer.write_many_sample(samples)
        elif event.key == ' ':
            with nidaqmx.Task() as task:
                task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
                task.write(5.0)
                task.stop()


    fig.canvas.mpl_connect('key_press_event', on_key_press)

    NIdev.calculateOffset()

    metric = calculate_metric()

    def udatePlot(x, y_l):
        xmax = np.max(x)
        ymin = np.min(y_l)
        ymax = np.max(y_l)

        ydelta = ymax - ymin
        ymin -= ydelta * .1
        ymax += ydelta * .1

        ax.set_xlim([min(0, xmax - 15), xmax])
        ax.set_ylim([ymin, ymax])

        for l, y in zip(lines, y_l):
            l.set_xdata(x)
            l.set_ydata(y)

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(.001)

    l = 150
    n = np.arange(0, 20) * 2 * np.pi / 21
    cx = np.cos(n) * l / 2  # array([1, 0, -1, 0]) * l / 2
    cy = np.sin(n) * l / 2  # array([0, 1, 0, -1]) * l / 2
    theta = np.arctan2(cy, cx)

    x = []
    y_l = [[], [], [], []]
    t0 = time()
    while (True):

        tmp = NIdev.getForceMetric()
        tmp = np.flip(tmp)

        timeHist.append(time() - t0)
        dataHist.append(tmp)
        (epsilon) = metric.calculate_metric_(tmp, cx, cy, theta)

        metricHist.append(epsilon)

        t = time() - t0
        x.append(t)
        for i in range(4):
            y_l[i].append(epsilon)

        udatePlot(x, y_l)

        print(tmp)


